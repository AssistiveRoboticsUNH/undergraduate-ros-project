import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from naoqi_bridge_msgs.msg import HeadTouch
class Demo(Node):
    def __init__(self):
        super().__init__('demo')
        self.publisher = self.create_publisher(String, 'speech', 10)
        self.subscriber = self.create_subscription(HeadTouch, 'head_touch', self.listener_callback, 10)
        self.subscriber

    def listener_callback(self, msg):
        # self.get_logger().info('I heard on topic_sub: "%s"' % msg.data)
        temp = String()
        temp.data = 'What?'
        self.publisher.publish(temp)
        self.get_logger().info('Publishing on topic_pub: "%s"' % temp.data)

def main(args=None):
    rclpy.init(args=args)

    demo = Demo()

    rclpy.spin(demo)

    demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()