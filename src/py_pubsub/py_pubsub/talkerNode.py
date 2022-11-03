import rclpy
from rclpy.node import Node

from std_msgs.msg import String


class TalkerNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, '/speech', 10)
        self.subscription = self.create_subscription(
            String,
            '/audio',
            self.listener_callback,
            10)

    def listener_callback(self, msg):
        self.get_logger().info('Info recieved')


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = TalkerNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()