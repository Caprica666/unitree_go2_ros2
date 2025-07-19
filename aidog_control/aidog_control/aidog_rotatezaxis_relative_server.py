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
# limitations under the License.import time

import time

from aidog_interfaces.action import RotateZAxisRelative
from geometry_msgs.msg import Twist
from rcl_interfaces.msg import SetParametersResult

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from threading import Event

class RotateZAxisRelativeServer(Node):

    def __init__(self):
        super().__init__('aidog_rotatezaxis_relative_server')
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self._action_server = ActionServer(
            self,
            RotateZAxisRelative,
            'rotatezaxis_relative',
            self.execute_callback,
            cancel_callback=self.cancel_callback)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.get_logger().info('Started RotateZAxisRelativeServer node')
        self.finish_event = Event()
        self.finish_event.clear()
        self.result = RotateZAxisRelative.Result()
        self.feedback_msg = RotateZAxisRelative.Feedback()

    def rotatezaxis_relative(self, angular_velocity):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = angular_velocity
        self.velocity_publisher.publish(twist)
        return
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.goal = goal_handle.request
        self.goal_handle = goal_handle
        angular_velocity = self.goal.angular_velocity
        self.current_angle = self.goal.start_angle
        self.elapsed = 0.0
        self.result.success = False
        self.result.atend = False
        duration = abs(goal_handle.request.turn_angle) / angular_velocity
        self.get_logger().info('Starting rotation: turn_angle {0} duration {1}'.format(self.goal.turn_angle, duration))
        self.feedback_msg = RotateZAxisRelative.Feedback()
        self.feedback_timer = self.create_timer(0.2, self.publish_feedback)
        self.result_timer = self.create_timer(duration, self.publish_result)
        self.feedback_timer.reset()
        self.result_timer.reset()
        self.rotatezaxis_relative(angular_velocity)
        self.finish_event.wait()
        
        self.finish_event.clear()
        self.get_logger().info('Returning result: last_angle {0} elapsed_time {1}'.format(self.current_angle, self.elapsed))
        if self.result.success:
            return self.result
        else:
            return CancelResponse.ACCEPT
    
    def publish_feedback(self):
        atend = ((self.goal.turn_angle > 0) & (self.current_angle >= self.goal.end_angle)) | \
                ((self.goal.turn_angle < 0) & (self.current_angle <= self.goal.end_angle))
        self.elapsed += self.feedback_timer.timer_period_ns / 1e9  # Convert nanoseconds to seconds
        self.current_angle += self.goal.turn_angle
        if atend:
            self.get_logger().info('Reached end angle: {0}'.format(self.current_angle))
            self.result.atend = True
            self.publish_result()
            return
        
        if self.goal_handle.is_cancel_requested:
            self.cancel_callback(self.goal)
            return
        
        self.feedback_msg.current_angle = self.current_angle
        self.feedback_msg.current_time = self.elapsed
        self.get_logger().info('Publishing feedback: current_angle {0} current_time {1}'.format(self.current_angle, self.elapsed))
        self.goal_handle.publish_feedback(self.feedback_msg)
        
    def publish_result(self):
        self.feedback_timer.cancel()
        self.result_timer.cancel()
        self.result.last_angle = self.current_angle
        self.result.elapsed_time = self.elapsed
        self.result.success = True
        self.result.message = 'Rotation successful'
        self.rotatezaxis_relative(0.0)  # Stop the robot after rotation
        self.goal_handle.succeed()
        self.finish_event.set()
               
    def cancel_callback(self, goal_handle):
        self.feedback_timer.cancel()
        self.result_timer.cancel()
        self.goal_handle.canceled()
        self.get_logger().info('Goal canceled')
        self.result.success = False
        self.result.message = 'ERROR: Rotation canceled'
        self.result.last_angle = self.current_angle
        self.result.elapsed_time = self.elapsed
        self.get_logger().info('Canceling goal...')
        return

def main(args=None):
    try:
        with rclpy.init(args=args):
            aidog_rotatezaxis_relative_server = RotateZAxisRelativeServer()
            executor = MultiThreadedExecutor()
            rclpy.spin(aidog_rotatezaxis_relative_server, executor=executor)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()