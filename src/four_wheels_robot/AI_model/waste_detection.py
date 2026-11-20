import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image


class WasteDetection(Node):
    def __init__(self):
        super().__init__("waste_detection")
        self.bridge = CvBridge()
        self.names_pub = self.create_publisher(
                String,
                "waste_found",10
                )
        self.image_sub = self.create_subscription(
                Image,
                "/camera/image_raw",
                self.process_images,
                10)

    def process_images(self, msg:Image):
        frame = self.bridge.imgmsg_to_cv2(msg,desired_encoding="bgr8")


        out = String()
        out.data = label 
        self.names_pub.publish(out)
        self.get_logger().info(f"Waste found: {label}")


def main():
    rclpy.init(args=None)
    node  =WasteDetection()
    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()




