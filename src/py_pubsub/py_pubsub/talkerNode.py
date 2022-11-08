from __future__ import print_function
import scipy.io.wavfile as wavf
import numpy as np

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from naoqi_bridge_msgs.msg import AudioBuffer
from naoqi_bridge_msgs.msg import HeadTouch


class TalkerNode(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher = self.create_publisher(String, '/speech', 10)
        self.subscription = self.create_subscription(AudioBuffer, '/audio', self.listener_callback, 10)
        self.subscription = self.create_subscription(HeadTouch, '/head_touch', self.head_touch_callback, 10)
        self.data = []
        self.listening = False

    def head_touch_callback(self, msg):
        self.get_logger().info('Head touched')

        if self.listening:
            self.listening = False
            temp = String()
            temp.data = 'I am done listening now.'
            self.publisher.publish(temp)
            self.interpret()

        else:
            self.listening = True
            temp = String()
            temp.data = 'I am listening now.'
            self.publisher.publish(temp)

    def listener_callback(self, msg):

        if self.listening:
            self.data.append(msg.data)
            self.get_logger().info('Info recieved')

    def interpret(self):
        frequency = 48000
        out_f = 'out.wav'
        wavf.write(out_f, frequency, np.array(self.data))


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
