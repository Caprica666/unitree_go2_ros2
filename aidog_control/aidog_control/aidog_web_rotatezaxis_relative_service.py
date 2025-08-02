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

from threading import Event
from aidog_interfaces.srv import RotateZAxisRelative
from geometry_msgs.msg import Twist
from ros2web_interfaces.srv import HTTP
from ros2web_interfaces.msg import ContentType, BodyPart
import json
import urllib.parse
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup 
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

    
class RotateZAxisRelativeService(Node):
    def __init__(self):
        super().__init__('aidog_rotatezaxis_relative_service')
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.callback_group = ReentrantCallbackGroup()
        self.srv = self.create_service(HTTP,
                                       'http/get/aidog_rotatezaxis_relative',
                                       self.handle_rotatezaxis,
                                       callback_group = self.callback_group)
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.get_logger().info('Started RotateZAxisRelativeService node')
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        

    def rotatezaxis_relative(self, angular_velocity):
        self.twist.angular.z = angular_velocity
        self.velocity_publisher.publish(self.twist)
    
    def make_response(self, jsonresponse, httpresponse, status_code=200):
        httpresponse.status = status_code
        httpresponse.content_type = ContentType.APPLICATION_JSON
        httpresponse.text = json.dumps(jsonresponse)
        return httpresponse
            
    def handle_rotatezaxis(self, request: HTTP.Request, response: HTTP.Response):
        query = dict(urllib.parse.parse_qsl(request.query))
        turn_angle = float(query.get('turn_angle', 0.2))
        angular_velocity = float(query.get('angular_velocity', 0.1))
        start_angle = float(query.get('start_angle', 0.0))
        end_angle = float(query.get('end_angle', 2.0))
        jsonresponse = { }
        jsonresponse['atend'] = False
        if turn_angle > 0:
            max_angle = end_angle - start_angle
            if max_angle < 0:
                jsonresponse['success'] = False
                jsonresponse['message'] = 'ERROR: turn_angle is positive but end_angle is less than start_angle'
                self.get_logger().error(jsonresponse.message)
                return self.make_response(jsonresponse, response, 400)
            if max_angle < turn_angle:
                turn_angle = max_angle
                jsonresponse['atend'] = True
        elif turn_angle < 0:
            min_angle = end_angle - start_angle
            if min_angle > 0:
                jsonresponse['success'] = False
                jsonresponse['message'] = 'ERROR: turn_angle is negative but end_angle is greater than start_angle'
                self.get_logger().error(response.message)
                return self.make_response(jsonresponse, response, 400)
            if min_angle > turn_angle:
                turn_angle = min_angle
                jsonresponse['atend'] = True            
    
        duration = abs(turn_angle) / angular_velocity
        self.get_logger().info('Starting rotation: turn_angle {0} duration {1}'.format(turn_angle, duration))
        self.finish_event = Event()
        self.rotatezaxis_relative(angular_velocity)
        self.result_timer = self.create_timer(duration, self.publish_result, callback_group=self.callback_group)
        self.finish_event.wait()
        
        jsonresponse['success'] = True
        jsonresponse['last_angle'] = start_angle + turn_angle
        jsonresponse['elapsed_time'] = duration
        jsonresponse['message'] = 'Rotation successful: last_angle {0} elapsed_time {1}'.format(jsonresponse['last_angle'], duration)
        self.finish_event.clear()
        self.get_logger().info('Returning response: last_angle {0} elapsed_time {1}'.format(jsonresponse['last_angle'], duration))
        return self.make_response(jsonresponse, response, 200)
        
    def publish_result(self):
        self.rotatezaxis_relative(0.0)  # Stop the robot after rotation
        self.get_logger().info('Stopping rotation')
        self.result_timer.cancel()
        self.finish_event.set()


def main(args=None):
    rclpy.init(args=args)
    aidog_rotatezaxis_relative_service = RotateZAxisRelativeService()
    executor = MultiThreadedExecutor()
    executor.add_node(aidog_rotatezaxis_relative_service)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        aidog_rotatezaxis_relative_service.rotatezaxis_relative(0.0)  # Ensure to stop the robot
        executor.remove_node(aidog_rotatezaxis_relative_service)
        aidog_rotatezaxis_relative_service.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()