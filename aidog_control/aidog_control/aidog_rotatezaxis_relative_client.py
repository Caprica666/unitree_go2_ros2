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

from aidog_interfaces.srv import RotateZAxisRelative

import rclpy

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import sys


class RotateZAxisRelativeClient(Node):

    def __init__(self, turn_angle=30.0, start_angle=0.0, angular_velocity=0.1, end_angle=180.0):
        super().__init__('rotatezaxis_relative_client')
        self.get_logger().info('Started RotateZAxisRelativeClient node')
        self.cli = self.create_client(RotateZAxisRelative, 'aidog_rotatezaxis_relative')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = RotateZAxisRelative.Request()
        self.req.turn_angle = turn_angle
        self.req.start_angle = start_angle
        self.req.angular_velocity = angular_velocity
        self.req.end_angle = end_angle

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info(future.result().message)
        else:
            self.get_logger().error('Service call failed.')

def main(args=None):
        rclpy.init(args=args)
        turn_angle = 30.0
        start_angle = 0.0
        end_angle = 180.0
        deg2rad = (3.14159 / 180)
        angular_velocity = 5  # degrees per second
        # Parse command-line arguments if provided
        if len(sys.argv) > 1:
            turn_angle = float(sys.argv[1])                         
        if len(sys.argv) > 2:               
            start_angle = float(sys.argv[2])
        if len(sys.argv) > 3:
            angular_velocity = float(sys.argv[3])
        if len(sys.argv) > 4:
            end_angle = float(sys.argv[4])
        turn_angle = turn_angle * deg2rad # Convert degrees to radians
        angular_velocity = angular_velocity * deg2rad  # Convert degrees per second to radians per second
        client = RotateZAxisRelativeClient(turn_angle, start_angle, angular_velocity, end_angle)
        client.send_request()

if __name__ == '__main__':
    main()