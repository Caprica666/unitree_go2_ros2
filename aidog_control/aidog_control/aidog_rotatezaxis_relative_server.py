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
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_system_default
from rclpy.service_introspection import ServiceIntrospectionState


class RotateZAxisRelativeServer(Node):

    def __init__(self):
        super().__init__('aidog_rotatezaxis_relative_server')
        self._action_server = ActionServer(
            self,
            RotateZAxisRelativeServer,
            'rotate_zaxis_relative',
            self.execute_callback,
            cancel_callback=self.cancel_callback)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.add_on_set_parameters_callback(self.on_set_parameters_callback)
        self.add_post_set_parameters_callback(self.on_post_set_parameters_callback)
        self.declare_parameter('action_server_configure_introspection', 'disabled')
        # TODO: get angular velocity from another Node
        self.angular_velocity = 0.1  # radians per second

    def _check_parameter(self, parameter_list, parameter_name):
        result = SetParametersResult()
        result.successful = True
        for param in parameter_list:
            if param.name != parameter_name:
                continue

            if param.type_ != Parameter.Type.FLOAT:
                result.successful = False
                result.reason = 'must be a number'
                break

            if param.value not in ('disabled', 'metadata', 'contents'):
                result.successful = False
                result.reason = "must be one of 'disabled', 'metadata', or 'contents"
                break

        return result

    def on_set_parameters_callback(self, parameter_list) -> SetParametersResult:
        return self._check_parameter(parameter_list, 'action_server_configure_introspection')

    def on_post_set_parameters_callback(self, parameter_list):
        for param in parameter_list:
            if param.name != 'action_server_configure_introspection':
                continue

            introspection_state = ServiceIntrospectionState.OFF
            if param.value == 'disabled':
                introspection_state = ServiceIntrospectionState.OFF
            elif param.value == 'metadata':
                introspection_state = ServiceIntrospectionState.METADATA
            elif param.value == 'contents':
                introspection_state = ServiceIntrospectionState.CONTENTS

            self._action_server.configure_introspection(self.get_clock(),
                                                        qos_profile_system_default,
                                                        introspection_state)
            break

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

        feedback_msg = RotateZAxisRelative.Feedback()
        feedback_msg.current_angle = goal_handle.request.start_angle
        feedback_msg.message = 'Starting rotation'
        angular_velocity = goal_handle.request.angular_velocity
        start_time = self.get_clock().now()
        elapsed = 0.0
        duration = goal_handle.request.turn_angle / angular_velocity
        self.rotatezaxis_relative(angular_velocity)
        while elapsed < duration:
            current_angle = elapsed * angular_velocity + goal_handle.request.start_angle
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                result = RotateZAxisRelative.Result()
                result.success = False
                result.end_angle = current_angle
                result.elapsed_time = self.elapsed
                result.message = 'ERROR: Rotation canceled'
                return result
            feedback_msg.message = 'Continuing rotation'
            feedback_msg.elapsed_time = self.elapsed
            feedback_msg.current_angle = current_angle
            self.get_logger().info('Feedback: current_angle {0}'.format(feedback_msg.current_angle))
            goal_handle.publish_feedback(feedback_msg)            
            time.sleep(0.1)
            now = self.get_clock().now()
            elapsed = (now - start_time).nanoseconds / 1e9  # seconds
        goal_handle.succeed()
        result = RotateZAxisRelative.Result()
        result.success = True
        result.end_angle = current_angle
        result.elapsed_time = elapsed
        result.message = 'Rotation successful'
        return result

    def cancel_callback(self, goal_handle):
        self.get_logger().info('Canceling goal...')
        return CancelResponse.ACCEPT

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