import rclpy
from rclpy.node import Node

import math
from datetime import *

from geometry_msgs.msg import Twist
from std_msgs.msg import String
from nav_msgs.msg import Odometry


class RotateNode(Node):

    def __init__(self):
        super().__init__('rotateNode')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(String, '/spin', self.spin_callback, 10)
        self.subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.initial_orientation = 0.0
        self.lastTouched = datetime.now()
        self.first = 0

    def spin_callback(self, msg):
        now = datetime.now()
        if self.lastTouched + timedelta(seconds=1) < now:
            self.first = 1
            self.lastTouched = now
            temp = Twist()
            temp.angular.z = 0.2
            self.publisher.publish(temp)
        self.lastTouched = now

    def odom_callback(self, msg):
        # self.get_logger().info("odom recieved")
        current_orientation = 2 * math.acos(msg.pose.pose.orientation.w)
        if self.first == 1:
            self.initial_orientation = current_orientation
            self.get_logger().info("Starting " + str(self.initial_orientation))
            self.first = 0
        else:
            now = datetime.now()
            difference = abs(self.initial_orientation) - abs(current_orientation)
            if abs(difference) < .1 and self.lastTouched + timedelta(seconds=3) < now:
                self.get_logger().info("Stopping: " + str(current_orientation))
                temp = Twist()
                temp.angular.z = 0.0
                self.publisher.publish(temp)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = RotateNode()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
