#!/usr/bin/env python3
# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# The program has a short runtime, so you can directly set the parameter
# "action_client_configure_introspection" at execution command
# e.g.
# ros2 run aidog_control rotatezaxis_relative_client --ros-args -p
# "action_client_configure_introspection:=contents"

from aidog_interfaces.action import RotateZAxisRelative

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import sys


class RotateZAxisRelativeClient(Node):

    def __init__(self):
        super().__init__('rotatezaxis_relative_client')
        self._action_client = ActionClient(self, RotateZAxisRelative, 'rotatezaxis_relative')
        self.get_logger().info('Started RotateZAxisRelativeClient node')

    def send_goal(self, turn_angle, start_angle, angular_velocity, end_angle):
        goal_msg = RotateZAxisRelative.Goal()
        goal_msg.turn_angle = turn_angle
        goal_msg.start_angle = start_angle
        goal_msg.angular_velocity = angular_velocity
        goal_msg.end_angle = end_angle
        self.get_logger().info('Sending goal with turn_angle: {0}, start_angle: {1}, angular_velocity: {2} end_angle: {3}'.format(
            goal_msg.turn_angle, goal_msg.start_angle, goal_msg.angular_velocity, goal_msg.end_angle))

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return
        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: last_angle = {0} elapsed_time = '.format(result.last_angle, result.elapsed_time))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: current_angle {0} current_time {1}'.format(feedback.current_angle, feedback.current_time))


def main(args=None):
    try:
        with rclpy.init(args=args):
            action_client = RotateZAxisRelativeClient()
            turn_angle = 30.0
            start_angle = 0.0
            end_angle = 180.0
            angular_velocity = 0.05  # radians per second
            # Parse command-line arguments if provided
            if len(sys.argv) > 1:
                turn_angle = float(sys.argv[1])                         
            if len(sys.argv) > 2:               
                start_angle = float(sys.argv[2])
            if len(sys.argv) > 3:
                angular_velocity = float(sys.argv[3])
            if len(sys.argv) > 4:
                end_angle = float(sys.argv[4])
            turn_angle = turn_angle / 180 # Convert degrees to radians
            action_client.send_goal(turn_angle, start_angle, angular_velocity, end_angle)
            rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()