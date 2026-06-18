import os
import sys
import math
import threading
import time
from typing import List, Optional

try:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from sensor_msgs.msg import JointState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    Node = object  # Dummy class to prevent NameError

import tkinter as tk
from tkinter import ttk
import tkinter.scrolledtext as st

# Ensure resources can be imported
sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from resources import kinematika2


# Constants for ROS
CMD_TOPIC = "/prarob/nl_command"
STATUS_TOPIC = "/prarob/agent_status"
JOINT_STATES_TOPIC = "/joint_states"
JOINT_NAMES = ["joint_1", "joint_2", "joint_3"]


if ROS2_AVAILABLE:
    class GuiNode(Node):
        def __init__(self, update_status_cb):
            super().__init__("prarob_gui_node")
            self.update_status_cb = update_status_cb
            
            # Publishers and Subscribers
            self.cmd_pub = self.create_publisher(String, CMD_TOPIC, 10)
            self.status_sub = self.create_subscription(String, STATUS_TOPIC, self._on_agent_status, 10)
            self.joint_pub = self.create_publisher(JointState, JOINT_STATES_TOPIC, 10)
            
            # Timer for publishing joint states to RVIZ (~30 Hz)
            self.timer = self.create_timer(1.0 / 30.0, self._publish_joint_states)
            
            # Current joint values in radians
            self.current_q1 = 0.0
            self.current_q2 = 0.0
            self.current_q3 = 0.0

        def _on_agent_status(self, msg: String):
            # Pass the message back to the GUI thread
            self.update_status_cb(msg.data)

        def send_command(self, text: str):
            msg = String()
            msg.data = text
            self.cmd_pub.publish(msg)

        def set_joints(self, q1: float, q2: float, q3: float):
            """Update internal joint states in radians"""
            self.current_q1 = q1
            self.current_q2 = q2
            self.current_q3 = q3

        def _publish_joint_states(self):
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = JOINT_NAMES
            msg.position = [self.current_q1, self.current_q2, self.current_q3]
            self.joint_pub.publish(msg)
else:
    GuiNode = None


class PrarobGUI(tk.Tk):
    def __init__(self):
        super().__init__()

        self.title("PRAROB Control Panel")
        self.geometry("800x600")
        
        try:
            import sv_ttk
            sv_ttk.set_theme("dark")
        except ImportError:
            print("sv_ttk not installed, using default theme.")

        # ROS2 Node initialization
        if ROS2_AVAILABLE:
            rclpy.init(args=None)
            self.ros_node = GuiNode(self.on_agent_status_received)
            
            # Start ROS2 spinning in a separate thread
            self.ros_thread = threading.Thread(target=self._spin_ros, daemon=True)
            self.ros_thread.start()
        else:
            self.ros_node = None
            print("[WARNING] rclpy not found! Running GUI in offline mode without ROS2 integration.")

        # Build UI
        self._build_ui()
        
        # Hardware state
        self.hw_port = None
        self.hw_packet = None
        
        # Initialize calculations
        self.update_dk_from_sliders()

    def _spin_ros(self):
        rclpy.spin(self.ros_node)

    def _build_ui(self):
        # Tabs configuration
        self.tabview = ttk.Notebook(self)
        self.tabview.pack(fill="both", expand=True, padx=20, pady=20)

        self.tab_manual = ttk.Frame(self.tabview)
        self.tab_draw = ttk.Frame(self.tabview)
        self.tab_auto = ttk.Frame(self.tabview)
        self.tabview.add(self.tab_manual, text="Manual Mode")
        self.tabview.add(self.tab_draw, text="Draw Mode")
        self.tabview.add(self.tab_auto, text="Autonomous Mode")

        self._build_manual_tab()
        self._build_draw_tab()
        self._build_auto_tab()

    def _build_manual_tab(self):
        # Split into DK and IK sections
        self.tab_manual.grid_columnconfigure(0, weight=1)
        self.tab_manual.grid_columnconfigure(1, weight=1)

        # === Direct Kinematics Frame ===
        self.dk_frame = ttk.LabelFrame(self.tab_manual, text="Direct Kinematics (Joint Angles)")
        self.dk_frame.grid(row=0, column=0, padx=10, pady=10, sticky="nsew")

        self.dk_sliders = []
        self.dk_entries = []
        joint_names = ["Base Rotation (°):", "Joint 1 (°):", "Joint 2 (°):"]
        
        for i, name in enumerate(joint_names):
            row_frame = ttk.Frame(self.dk_frame)
            row_frame.pack(fill="x", padx=10, pady=5)
            
            ttk.Label(row_frame, text=name, width=15, anchor="w").pack(side="left")
            
            slider = tk.Scale(row_frame, from_=-180, to=180, orient="horizontal", command=lambda v, idx=i: self.on_slider_change(idx, v), showvalue=0)
            slider.set(0)
            slider.pack(side="left", fill="x", expand=True, padx=10)
            self.dk_sliders.append(slider)
            
            entry = ttk.Entry(row_frame, width=8)
            entry.insert(0, "0.0")
            entry.bind("<Return>", lambda e, idx=i: self.on_dk_entry_change(idx))
            entry.pack(side="left")
            self.dk_entries.append(entry)

        self.dk_result_label = ttk.Label(self.dk_frame, text="End Effector: X=0.0, Y=0.0, Z=0.0 mm", font=("TkDefaultFont", 10, "bold"))
        self.dk_result_label.pack(pady=15)

        self.dk_move_btn = ttk.Button(self.dk_frame, text="Move to Joints", command=self.move_from_dk)
        self.dk_move_btn.pack(pady=10)


        # === Inverse Kinematics Frame ===
        self.ik_frame = ttk.LabelFrame(self.tab_manual, text="Inverse Kinematics (End Effector)")
        self.ik_frame.grid(row=0, column=1, padx=10, pady=10, sticky="nsew")

        self.ik_entries = []
        coords = ["X (mm):", "Y (mm):", "Z (mm):"]
        default_vals = ["150.0", "0.0", "0.0"] # Arbitrary reachable default
        
        for i, name in enumerate(coords):
            row_frame = ttk.Frame(self.ik_frame)
            row_frame.pack(fill="x", padx=10, pady=5)
            
            ttk.Label(row_frame, text=name, width=10, anchor="w").pack(side="left")
            
            entry = ttk.Entry(row_frame)
            entry.insert(0, default_vals[i])
            entry.bind("<Return>", lambda e: self.update_ik_calculation())
            entry.pack(side="left", fill="x", expand=True, padx=10)
            self.ik_entries.append(entry)

        self.ik_calc_btn = ttk.Button(self.ik_frame, text="Calculate IK", command=self.update_ik_calculation)
        self.ik_calc_btn.pack(pady=10)

        self.ik_result_label = ttk.Label(self.ik_frame, text="Joints: q1=0.0°, q2=0.0°, q3=0.0°", font=("TkDefaultFont", 10, "bold"))
        self.ik_result_label.pack(pady=5)
        
        self.ik_error_label = ttk.Label(self.ik_frame, text="", foreground="red")
        self.ik_error_label.pack(pady=0)

        self.ik_move_btn = ttk.Button(self.ik_frame, text="Move to Position", command=self.move_from_ik)
        self.ik_move_btn.pack(pady=10)

    def _build_auto_tab(self):
        self.tab_auto.grid_columnconfigure(0, weight=1)
        self.tab_auto.grid_rowconfigure(1, weight=1)

        # Command input area
        cmd_frame = ttk.Frame(self.tab_auto)
        cmd_frame.grid(row=0, column=0, sticky="ew", pady=(0, 10))

        self.cmd_entry = ttk.Entry(cmd_frame)
        self.cmd_entry.pack(side="left", fill="x", expand=True, padx=(0, 10))
        self.cmd_entry.bind("<Return>", lambda e: self.execute_auto_command())

        self.exec_btn = ttk.Button(cmd_frame, text="Execute", command=self.execute_auto_command)
        self.exec_btn.pack(side="right")

        # Status log area
        self.status_log = st.ScrolledText(self.tab_auto, font=("Consolas", 10))
        self.status_log.grid(row=1, column=0, sticky="nsew")
        self.status_log.insert("end", "Waiting for agent status...\n")
        self.status_log.configure(state="disabled")

    def _build_draw_tab(self):
        self.tab_draw.grid_columnconfigure(0, weight=1)
        self.tab_draw.grid_rowconfigure(1, weight=1)

        input_frame = ttk.Frame(self.tab_draw)
        input_frame.grid(row=0, column=0, sticky="ew", padx=10, pady=(10, 10))

        self.draw_entries = {}
        fields1 = [("X Start", "150.0"), ("Y Start", "150.0"), 
                   ("X End", "150.0"), ("Y End", "-150.0")]
        fields2 = [("Points", "20"), ("Pause (s)", "0.3")]
        
        row1 = ttk.Frame(input_frame)
        row1.pack(fill="x", pady=5)
        row2 = ttk.Frame(input_frame)
        row2.pack(fill="x", pady=5)
        
        for label_text, default_val in fields1:
            ttk.Label(row1, text=label_text).pack(side="left", padx=(10, 2))
            entry = ttk.Entry(row1, width=8)
            entry.insert(0, default_val)
            entry.pack(side="left", padx=(0, 10))
            self.draw_entries[label_text] = entry
            
        for label_text, default_val in fields2:
            ttk.Label(row2, text=label_text).pack(side="left", padx=(10, 2))
            entry = ttk.Entry(row2, width=8)
            entry.insert(0, default_val)
            entry.pack(side="left", padx=(0, 10))
            self.draw_entries[label_text] = entry

        self.draw_btn = ttk.Button(row2, text="Draw Line", command=self.execute_draw_command)
        self.draw_btn.pack(side="left", padx=20)

        self.draw_log = st.ScrolledText(self.tab_draw, font=("Consolas", 10))
        self.draw_log.grid(row=1, column=0, sticky="nsew", padx=10, pady=(0, 10))
        self.draw_log.insert("end", "Ready to draw.\n")
        self.draw_log.configure(state="disabled")

    # --- Draw Mode Logic ---

    def _append_draw_log(self, text: str):
        self.draw_log.configure(state="normal")
        self.draw_log.insert("end", f"{text}\n")
        self.draw_log.see("end")
        self.draw_log.configure(state="disabled")

    def execute_draw_command(self):
        try:
            x_start = float(self.draw_entries["X Start"].get())
            y_start = float(self.draw_entries["Y Start"].get())
            x_end = float(self.draw_entries["X End"].get())
            y_end = float(self.draw_entries["Y End"].get())
            broj_tocaka = int(self.draw_entries["Points"].get())
            pauza = float(self.draw_entries["Pause (s)"].get())
        except ValueError:
            self._append_draw_log("Error: Invalid input values. Please enter valid numbers.")
            return

        self.draw_btn.configure(state="disabled")
        threading.Thread(target=self._draw_task, args=(x_start, y_start, x_end, y_end, broj_tocaka, pauza), daemon=True).start()

    def _draw_task(self, x_start, y_start, x_end, y_end, broj_tocaka, pauza):
        try:
            tocke = [(x_start + (i / broj_tocaka) * (x_end - x_start),
                      y_start + (i / broj_tocaka) * (y_end - y_start))
                     for i in range(broj_tocaka + 1)]

            # Check all points first
            for i, (x, y) in enumerate(tocke):
                try:
                    kinematika2.izracunaj_IK(x, y, z=0)
                except ValueError:
                    self.after(0, self._append_draw_log, f"Point {i+1} ({x:.1f}, {y:.1f}) is out of reach! Aborting.")
                    return

            self.after(0, self._append_draw_log, f"Drawing line from ({x_start}, {y_start}) to ({x_end}, {y_end})...")

            if not self.hw_port:
                self.hw_port, self.hw_packet = kinematika2.inicijaliziraj_motore()

            for i, (x, y) in enumerate(tocke):
                q1_rad, q2_rad, q3_rad = kinematika2.izracunaj_IK(x, y, z=0)
                
                # Update ROS (RVIZ) if available
                if self.ros_node:
                    self.ros_node.set_joints(q1_rad, q2_rad, q3_rad)

                enc1 = kinematika2.kut_u_enkoder(1, math.degrees(q1_rad))
                enc2 = kinematika2.kut_u_enkoder(2, math.degrees(q2_rad))
                enc3 = kinematika2.kut_u_enkoder(3, math.degrees(q3_rad))
                
                self.hw_packet.write4ByteTxRx(self.hw_port, 1, kinematika2.ADDR_GOAL_POSITION, enc1)
                self.hw_packet.write4ByteTxRx(self.hw_port, 2, kinematika2.ADDR_GOAL_POSITION, enc2)
                self.hw_packet.write4ByteTxRx(self.hw_port, 3, kinematika2.ADDR_GOAL_POSITION, enc3)
                
                self.after(0, self._append_draw_log, f"  {i+1}/{len(tocke)}: X={x:.1f} Y={y:.1f}  ->  [{enc1}, {enc2}, {enc3}]")
                time.sleep(pauza)
                
            self.after(0, self._append_draw_log, "Done.")

        except Exception as e:
            self.after(0, self._append_draw_log, f"Hardware Error: {e}")
        finally:
            self.after(0, lambda: self.draw_btn.configure(state="normal"))

    # --- Manual Mode Logic ---

    def on_slider_change(self, idx, value):
        self.dk_entries[idx].delete(0, "end")
        self.dk_entries[idx].insert(0, f"{float(value):.1f}")
        self.update_dk_from_sliders()

    def on_dk_entry_change(self, idx):
        try:
            val = float(self.dk_entries[idx].get())
            self.dk_sliders[idx].set(val)
            self.update_dk_from_sliders()
        except ValueError:
            pass # Ignore invalid input

    def update_dk_from_sliders(self):
        try:
            q_deg = [float(e.get()) for e in self.dk_entries]
        except ValueError:
            return # Ignore if empty
        q_rad = [math.radians(deg) for deg in q_deg]
        
        # Update ROS node for RVIZ
        if self.ros_node:
            self.ros_node.set_joints(q_rad[0], q_rad[1], q_rad[2])

        # Calculate End Effector position
        x, y, z = kinematika2.izracunaj_DK(q_rad[0], q_rad[1], q_rad[2])
        self.dk_result_label.configure(text=f"End Effector: X={x:.1f}, Y={y:.1f}, Z={z:.1f} mm")

    def move_from_dk(self):
        try:
            q_deg = [float(e.get()) for e in self.dk_entries]
        except ValueError:
            return
        self._execute_hardware_move(q_deg[0], q_deg[1], q_deg[2])

    def update_ik_calculation(self) -> Optional[List[float]]:
        try:
            x = float(self.ik_entries[0].get())
            y = float(self.ik_entries[1].get())
            z = float(self.ik_entries[2].get())
            
            q1_rad, q2_rad, q3_rad = kinematika2.izracunaj_IK(x, y, z)
            q1_deg, q2_deg, q3_deg = math.degrees(q1_rad), math.degrees(q2_rad), math.degrees(q3_rad)
            
            self.ik_result_label.configure(text=f"Joints: q1={q1_deg:.1f}°, q2={q2_deg:.1f}°, q3={q3_deg:.1f}°")
            self.ik_error_label.configure(text="")
            
            # Optionally update RVIZ immediately on successful calc
            if self.ros_node:
                self.ros_node.set_joints(q1_rad, q2_rad, q3_rad)
            # And update sliders to match
            for i, deg in enumerate([q1_deg, q2_deg, q3_deg]):
                self.dk_sliders[i].set(deg)
                self.dk_entries[i].delete(0, "end")
                self.dk_entries[i].insert(0, f"{deg:.1f}")
            self.update_dk_from_sliders()
            
            return [q1_deg, q2_deg, q3_deg]
        except ValueError as e:
            self.ik_error_label.configure(text=str(e))
            self.ik_result_label.configure(text="Joints: N/A")
            return None

    def move_from_ik(self):
        q_deg = self.update_ik_calculation()
        if q_deg is not None:
            self._execute_hardware_move(q_deg[0], q_deg[1], q_deg[2])

    def _execute_hardware_move(self, q1, q2, q3):
        # Run hardware move in a separate thread to avoid blocking GUI
        def move_task():
            try:
                if not self.hw_port:
                    self.hw_port, self.hw_packet = kinematika2.inicijaliziraj_motore()
                
                kinematika2.pomakni_na_tocku(self.hw_port, self.hw_packet, q1, q2, q3, pauza=1.0)
                # Success
            except Exception as e:
                print(f"Hardware Error: {e}")
                
        threading.Thread(target=move_task, daemon=True).start()

    # --- Autonomous Mode Logic ---

    def execute_auto_command(self):
        cmd_text = self.cmd_entry.get().strip()
        if not cmd_text:
            return

        # Disable button until done (optional, based on agent status)
        self.exec_btn.configure(state="disabled")
        self.cmd_entry.delete(0, "end")

        # Publish to ROS
        if self.ros_node:
            self.ros_node.send_command(cmd_text)
        else:
            self._append_log(f"Offline mode: Command '{cmd_text}' logged, but not sent to ROS.")
            self.exec_btn.configure(state="normal")

    def on_agent_status_received(self, text: str):
        # Called from ROS thread, update GUI safely
        self.after(0, self._append_log, text)

    def _append_log(self, text: str):
        self.status_log.configure(state="normal")
        self.status_log.insert("end", f"> {text}\n")
        self.status_log.see("end")
        self.status_log.configure(state="disabled")

        if text.startswith("Done") or text.startswith("ERROR"):
            self.exec_btn.configure(state="normal")

    def on_closing(self):
        # Shutdown HW
        if self.hw_port and self.hw_packet:
            try:
                kinematika2.ugasi_motore(self.hw_port, self.hw_packet)
            except Exception:
                pass
        # Shutdown ROS
        if ROS2_AVAILABLE:
            try:
                rclpy.shutdown()
            except Exception:
                pass
        self.destroy()

if __name__ == "__main__":
    app = PrarobGUI()
    app.protocol("WM_DELETE_WINDOW", app.on_closing)
    app.mainloop()