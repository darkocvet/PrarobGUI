"""
nl_agent_node.py
----------------
YOUR main deliverable: the ROS2 node that hosts the LLM (ROSA) brain and bridges
it to Darko's GUI.

Data flow
---------
    [GUI text box, autonomous mode]
        --(std_msgs/String on /prarob/nl_command)-->  THIS NODE
        THIS NODE runs ROSA.invoke(command)  ->  connect_and_avoid tool runs
        --(std_msgs/String on /prarob/agent_status)--> [GUI status area]

The actual command processing runs in a background worker thread so a long
drawing/LLM call never blocks ROS callbacks and status keeps flowing.

Run with:
    ros2 run prarob_interact nl_agent      (after adding the entry point, see README)
"""

import os
import sys
import math
import queue
import threading
import time

import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState

# Locate kinematika2.py by walking up from this file's directory.
# Works whether running from source or from a colcon-installed entry point.
def _find_resources_dir():
    d = os.path.dirname(os.path.abspath(__file__))
    for _ in range(15):
        if os.path.isfile(os.path.join(d, 'kinematika2.py')):
            return d
        parent = os.path.dirname(d)
        if parent == d:
            break
        d = parent
    raise FileNotFoundError("Cannot locate kinematika2.py in any parent directory")

_RESOURCES_DIR = _find_resources_dir()
if _RESOURCES_DIR not in sys.path:
    sys.path.insert(0, _RESOURCES_DIR)
import kinematika2

from dotenv import load_dotenv
from langchain_openai import ChatOpenAI
from rosa import ROSA

from .prompts import get_prompts
from . import drawing_tools
from .drawing_tools import DRAWING_TOOLS, RobotContext

try:
    from yolo_msgs.msg import DetectionArray
except ImportError:
    DetectionArray = None

# Topic names -- THESE ARE THE CONTRACT WITH THE GUI. Agree them with Darko.
CMD_TOPIC = "/prarob/nl_command"     # GUI  -> agent   (std_msgs/String)
STATUS_TOPIC = "/prarob/agent_status"  # agent -> GUI   (std_msgs/String)


class NLAgentNode(Node):
    def __init__(self):
        super().__init__("nl_agent_node")

        load_dotenv()  # reads OPENAI_API_KEY from .env

        # --- ROS interface to the GUI -------------------------------------
        self.status_pub = self.create_publisher(String, STATUS_TOPIC, 10)
        self.create_subscription(String, CMD_TOPIC, self._on_command, 10)

        # --- robot context used by the drawing tool -----------------------
        self.ctx = RobotContext(self)
        drawing_tools.set_context(self.ctx)

        # --- hardware state for kinematika2 motor control ------------------
        self.hw_port = None
        self.hw_packet = None

        # --- YOLO detections ----------------------------------------------
        self.latest_detections = []
        if DetectionArray:
            self.create_subscription(DetectionArray, '/yolo/detections', self._on_yolo_detections, 10)
        else:
            self.get_logger().warn("yolo_msgs not installed, running without live YOLO")

        # --- the LLM + ROSA agent -----------------------------------------
        model = os.getenv("ROSA_MODEL", "gpt-4o-mini")
        self.llm = ChatOpenAI(model=model, temperature=0.0)
        self.agent = ROSA(
            ros_version=2,
            llm=self.llm,
            tools=DRAWING_TOOLS,
            prompts=get_prompts(),
            streaming=False,
        )

        # --- worker thread so invoke() does not block ROS -----------------
        self._jobs: "queue.Queue[str]" = queue.Queue()
        self._worker = threading.Thread(target=self._work_loop, daemon=True)
        self._worker.start()

        self.publish_status("Autonomous agent ready. Send a command.")
        self.get_logger().info(f"NL agent up. Model={model}. Listening on {CMD_TOPIC}.")

    # ----------------------------------------------------------------------
    #  GUI bridge
    # ----------------------------------------------------------------------
    def _on_command(self, msg: String):
        text = msg.data.strip()
        if text:
            self.publish_status(f"Received: \"{text}\"")
            self._jobs.put(text)

    def publish_status(self, text: str):
        self.status_pub.publish(String(data=text))

    def _work_loop(self):
        while rclpy.ok():
            try:
                command = self._jobs.get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                self.publish_status("Thinking...")
                answer = self.agent.invoke(command)
                self.publish_status(str(answer))
            except Exception as exc:  # keep the node alive on any failure
                self.get_logger().error(f"Agent error: {exc}")
                self.publish_status(f"ERROR: {exc}")

    # ======================================================================
    #  Seam to teammates' modules. Replace the fallbacks with real calls.
    # ======================================================================

    # VISION (teammate). Should query YOLO and return detections for the
    # requested classes. Format per item:
    #   {"class": str, "cx": float, "cy": float, "bbox": (x0,y0,x1,y1)}  # px
    
    def _on_yolo_detections(self, msg):
        current_dets = []
        for det in msg.detections:
            cx = det.bbox.center.position.x
            cy = det.bbox.center.position.y
            w = det.bbox.size.x
            h = det.bbox.size.y
            current_dets.append({
                "class": det.class_name,
                "cx": cx,
                "cy": cy,
                "bbox": (cx - w/2, cy - h/2, cx + w/2, cy + h/2)
            })
        self.latest_detections = current_dets

    def get_yolo_detections(self, class_names):
        if not class_names:
            return self.latest_detections
            
        filtered = []
        for d in self.latest_detections:
            if d["class"] in class_names:
                filtered.append(d)
        
        if not filtered and self.latest_detections:
            self.get_logger().warn(f"YOLO running, but {class_names} not found. Found: {[d['class'] for d in self.latest_detections]}")
        return filtered

    # CALIBRATION. Pixel -> robot base XY in mm.
    def image_to_base(self, cx, cy):
        # The robot base is positioned at the bottom center of the camera view.
        # Assuming a 640x480 image from the camera:
        base_cx = 320.0
        
        # The physical robot base is slightly below the bottom edge of the camera frame
        base_cy = 530.0 
        
        # Scale factor (pixels to mm). Based on an A4 paper occupying ~370 pixels in width
        scale = 0.8
        
        # Forward distance from robot base (X axis moves UP in the image -> decreasing cy)
        x_mm = (base_cy - cy) * scale
        
        # Left distance from robot base (Y axis moves LEFT in the image -> decreasing cx)
        y_mm = (base_cx - cx) * scale
        
        return (x_mm, y_mm)

    # KINEMATICS / EXECUTION — uses kinematika2 for IK + direct motor control.
    def draw_polyline(self, points_mm):
        """Move the marker tip along the given XY polyline (mm, in robot base frame).

        Uses kinematika2.izracunaj_IK for inverse kinematics and drives the
        Dynamixel motors directly through kinematika2's motor-control functions.
        Also publishes joint states so RViz stays in sync.
        """
        WAYPOINT_PAUSE = 0.3  # seconds between waypoints (reduced for A* trajectories)

        # --- 1. Validate all points are reachable BEFORE moving ---------------
        valid_points = []
        ik_results = []  # list of (q1_rad, q2_rad, q3_rad)
        for i, (x_mm, y_mm) in enumerate(points_mm):
            self.get_logger().info(f"Target point {i}: X={x_mm:.1f}, Y={y_mm:.1f}")
            try:
                q1_rad, q2_rad, q3_rad = kinematika2.izracunaj_IK(x_mm, y_mm, z=0)
                ik_results.append((q1_rad, q2_rad, q3_rad))
                valid_points.append((x_mm, y_mm))
            except ValueError as e:
                self.get_logger().warn(
                    f"[draw] waypoint {i} ({x_mm:.1f}, {y_mm:.1f}) unreachable: {e}. Skipping."
                )
                
        if not valid_points:
            self.publish_status("ERROR: No reachable waypoints in path.")
            return

        points_mm = valid_points

        # --- 2. Initialise motors (lazy, once) --------------------------------
        try:
            if self.hw_port is None:
                self.hw_port, self.hw_packet = kinematika2.inicijaliziraj_motore()
                self.get_logger().info("[draw] Motors initialised via kinematika2.")
        except Exception as e:
            self.get_logger().error(f"[draw] Motor init failed: {e}")
            self.publish_status(f"ERROR: could not initialise motors — {e}")
            return

        # --- 3. Execute each waypoint -----------------------------------------
        for i, ((x_mm, y_mm), (q1_rad, q2_rad, q3_rad)) in enumerate(
            zip(points_mm, ik_results)
        ):
            q1_deg = math.degrees(q1_rad)
            q2_deg = math.degrees(q2_rad)
            q3_deg = math.degrees(q3_rad)

            enc1 = kinematika2.kut_u_enkoder(1, q1_deg)
            enc2 = kinematika2.kut_u_enkoder(2, q2_deg)
            enc3 = kinematika2.kut_u_enkoder(3, q3_deg)

            self.hw_packet.write4ByteTxRx(
                self.hw_port, 1, kinematika2.ADDR_GOAL_POSITION, enc1
            )
            self.hw_packet.write4ByteTxRx(
                self.hw_port, 2, kinematika2.ADDR_GOAL_POSITION, enc2
            )
            self.hw_packet.write4ByteTxRx(
                self.hw_port, 3, kinematika2.ADDR_GOAL_POSITION, enc3
            )

            self.get_logger().info(
                f"[draw] waypoint {i+1}/{len(points_mm)}: "
                f"X={x_mm:.1f} Y={y_mm:.1f} -> enc [{enc1}, {enc2}, {enc3}]"
            )

            # Publish joint states so RViz follows along
            js = JointState()
            js.header.stamp = self.get_clock().now().to_msg()
            js.name = ['joint_1', 'joint_2', 'joint_3']
            js.position = [q1_rad, q2_rad, q3_rad]
            # Use the status publisher topic only if a JointState publisher is added
            # For now, just log — RViz integration can be added later.

            time.sleep(WAYPOINT_PAUSE)

        self.get_logger().info("[draw] Polyline complete.")

    def destroy_node(self):
        """Shut down motors cleanly before destroying the ROS node."""
        if self.hw_port is not None and self.hw_packet is not None:
            try:
                kinematika2.ugasi_motore(self.hw_port, self.hw_packet)
                self.get_logger().info("Motors shut down via kinematika2.")
            except Exception as e:
                self.get_logger().warn(f"Error shutting down motors: {e}")
            self.hw_port = None
            self.hw_packet = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = NLAgentNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
