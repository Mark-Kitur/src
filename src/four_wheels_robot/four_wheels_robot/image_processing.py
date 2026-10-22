# fake_detector.py
import random
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

DETECTED_OBJECTS = ["plastic", "organic", "metal","closed_gate", "open_gate"]


class FakeDetector(Node):
    def __init__(self):
        super().__init__("fake_detector")
        self.pub = self.create_publisher(String, "/class_detected", 10)
        self.create_timer(1.0, self.publish_random)

    def publish_random(self):
        msg = String()
        msg.data = random.choice(DETECTED_OBJECTS)
        self.pub.publish(msg)
        self.get_logger().info(f"Published: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = FakeDetector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()