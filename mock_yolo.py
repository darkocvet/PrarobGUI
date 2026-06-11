import rclpy
from rclpy.node import Node
import time

try:
    from yolo_msgs.msg import DetectionArray, Detection
except ImportError:
    print("ERROR: yolo_msgs not found. Make sure you 'source activate.sh' before running this script.")
    exit(1)

class MockYolo(Node):
    def __init__(self):
        super().__init__('mock_yolo_camera')
        self.pub = self.create_publisher(DetectionArray, '/yolo/detections', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.get_logger().info("=== MOCK YOLO CAMERA ACTIVE ===")
        self.get_logger().info("Publishing virtual 'traffic light', 'sports ball', and 'person' (obstacle)...")
        self.get_logger().info("Leave this running and use the GUI to send your command.")

    def timer_callback(self):
        msg = DetectionArray()
        
        # 1. Fake Traffic Light at pixel (100, 100)
        d1 = Detection()
        d1.class_name = "traffic light"
        d1.bbox.center.position.x = 100.0
        d1.bbox.center.position.y = 100.0
        d1.bbox.size.x = 50.0
        d1.bbox.size.y = 50.0
        
        # 2. Fake Sports Ball at pixel (400, 300)
        d2 = Detection()
        d2.class_name = "sports ball"
        d2.bbox.center.position.x = 400.0
        d2.bbox.center.position.y = 300.0
        d2.bbox.size.x = 40.0
        d2.bbox.size.y = 40.0

        # 3. Fake Obstacle: a person right in the middle at (250, 200)
        # This will force the LLM planner to draw a detour!
        d3 = Detection()
        d3.class_name = "person"
        d3.bbox.center.position.x = 250.0
        d3.bbox.center.position.y = 200.0
        d3.bbox.size.x = 80.0
        d3.bbox.size.y = 150.0
        
        msg.detections = [d1, d2, d3]
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MockYolo()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
