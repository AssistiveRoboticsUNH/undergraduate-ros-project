import math
import time
import sys

from shr_msgs.action import RecognizeRequest
from shr_msgs.action import RotateRequest
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node


class MinimalClientAsync(Node):

    def __init__(self):
        super().__init__('minimal_client_async')
        self.cli = ActionClient(self, RecognizeRequest, 'recognize_face')
        self.cli2 = ActionClient(self, RotateRequest, 'rotate')

    def send_spin_goal(self):
        goal_msg = RotateRequest.Goal()
        goal_msg.total_time = 20.0
        goal_msg.angle = math.pi * 2.5
        self.cli2.wait_for_server()

        return self.cli2.send_goal_async(goal_msg)

    def send_recognize_goal(self):
        goal_msg = RecognizeRequest.Goal()

        self.cli.wait_for_server()

        return self.cli.send_goal_async(goal_msg)



def main():
    rclpy.init()
    time.sleep(5.0)

    action_client = MinimalClientAsync()

    future = action_client.send_spin_goal()
    future1 = action_client.send_recognize_goal()

    rclpy.spin_until_future_complete(action_client, future1)
    rclpy.spin_until_future_complete(action_client, future)


if __name__ == '__main__':
    main()