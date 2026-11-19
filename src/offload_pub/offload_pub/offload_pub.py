import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray


class Publisher(Node):
    def __init__(self):
        super().__init__("Offload_Publisher")
        self.publisher_ = self.create_publisher(Float64MultiArray, "/offload_controller/commands", 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [0.0]
        self.publisher_.publish(msg)
        self.get_logger().info(f"Sending Messsage {msg.data}")


def main():
    rclpy.init()
    pub = Publisher()
    rclpy.spin(pub)
    pub.destroy_node()
    rclpy.shutdown()

