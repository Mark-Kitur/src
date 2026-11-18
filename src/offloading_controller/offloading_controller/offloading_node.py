import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from std_srvs.srv import SetBool


class OffloadingNode(Node):

    def __init__(self):
        super().__init__('offloading_node')

        # Publisher for communicating with the offloading hardware
        self.publisher = self.create_publisher(
            Int32,
            '/offloading/command',
            10
        )

        # Service through which other nodes request extension or retraction
        self.service = self.create_service(
            SetBool,
            '/offloading/control',
            self.control_callback
        )

        self.get_logger().info('Offloading node ready.')

    # Publishes commands: 0 = extend, 1 = retract
    def publish_command(self, command):
        msg = Int32()
        msg.data = command
        self.publisher.publish(msg)

        self.get_logger().info(
            f'Published offloading command: {command}'
        )

    # Handles extension and retraction requests
    def control_callback(self, request, response):
        if request.data:
            command = 0
            action = 'Extension'
        else:
            command = 1
            action = 'Retraction'

        self.publish_command(command)

        response.success = True
        response.message = f'{action} command published.'

        return response


def main(args=None):
    rclpy.init(args=args)
    node = OffloadingNode()

    try:
        # Keep the node active to receive service requests
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()