#!/usr/bin/env python3

from aidog_interfaces.action import RotateZAxisRelative
from aidog_control.aidog_rotatezaxis import RotateZAxis
from geometry_msgs.msg import Twist

import rclpy
from rclpy.action import ActionServer, CancelResponse
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from threading import Event

class RotateZAxisRelativeAction(RotateZAxis):

    def __init__(self):
        super().__init__('aidog_rotatezaxis_relative_action')
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])

        self._action_server = ActionServer(
            self,
            RotateZAxisRelative,
            'aidog_rotatezaxis_relative_action_server',
            self.execute_callback,
            cancel_callback=self.cancel_callback)
        self.result = RotateZAxisRelative.Result()
        self.response = None
        self.feedback_msg = RotateZAxisRelative.Feedback()
        self.feedback_timer = self.create_timer(0.2, self.publish_feedback, autostart=False)
        
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')
        self.goal = goal_handle.request
        self.goal_handle = goal_handle
        angular_velocity = self.goal.angular_velocity
        self.current_angle = self.goal.start_angle
        self.elapsed = 0.0
        self.result.success = False
        self.result.at_end = False
        self.duration = abs(goal_handle.request.turn_angle) / angular_velocity
        self.get_logger().info('Starting rotation: turn_angle {0} duration {1}'.format(self.goal.turn_angle, self.duration))
        self.feedback_msg = RotateZAxisRelative.Feedback()     
        self.feedback_timer.reset()

        response = self.handle_rotatezaxis(self.goal.turn_angle, self.goal.angular_velocity, self.goal.start_angle, self.goal.end_angle)
        self.result.message = response['message']
        if not response['success']:
            self.result.message = response['message']
            self.goal_handle.abort()
        return self.result
    
    def publish_feedback(self):
        dt = self.feedback_timer.timer_period_ns / 1e9  # Convert nanoseconds to seconds
        self.elapsed += dt
        self.current_angle += (self.goal.turn_angle * dt) / self.duration  
        if self.goal_handle.is_cancel_requested:
            self.cancel_callback(self.goal_handle)
            return      
        self.feedback_msg.current_angle = self.current_angle
        self.feedback_msg.current_time = self.elapsed
        #self.get_logger().info('Publishing feedback: current_angle {0} current_time {1}'.format(self.current_angle, self.elapsed))
        self.goal_handle.publish_feedback(self.feedback_msg)
        
    def publish_result(self):
        if self.feedback_timer is not None:
            self.feedback_timer.cancel()
        if self.response is not None:
            self.result.success = self.response['success']
            self.result.last_angle = self.response['last_angle']
            self.result.elapsed_time = self.response['elapsed_time']
            self.result.at_end = self.response['at_end']
            self.result.message = self.response['message']
        super().publish_result()
        self.goal_handle.succeed()
               
    def cancel_callback(self, goal_handle):
        if self.feedback_timer is not None:
            self.feedback_timer.cancel()
        self.get_logger().info('Goal canceled')
        self.result.success = False
        if self.result.message is None:
            self.result.message = 'error: Rotation canceled'
        self.result.last_angle = self.current_angle
        self.result.elapsed_time = self.elapsed
        self.get_logger().info('Canceling goal...')
        goal_handle.canceled()
        super().publish_result()
        return CancelResponse.ACCEPT

def main(args=None):
    try:
        with rclpy.init(args=args):
            aidog_rotatezaxis_relative_server = RotateZAxisRelativeAction()
            executor = MultiThreadedExecutor()
            rclpy.spin(aidog_rotatezaxis_relative_server, executor=executor)
            aidog_rotatezaxis_relative_server.get_logger().info('RotateZAxisRelativeAction exiting...')
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()