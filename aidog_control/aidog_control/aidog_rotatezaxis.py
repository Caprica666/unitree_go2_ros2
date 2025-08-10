#!/usr/bin/env python3

from threading import Event
from geometry_msgs.msg import Twist
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
    
class RotateZAxis(Node):
    def __init__(self, name, absolute=False):
        super().__init__(name)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.velocity_publisher = self.create_publisher(Twist, 'cmd_vel', 1)
        self.get_logger().info('Started RotateZAxis')
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.deg2rad = 3.141592653589793 / 180.0
        self.absolute = absolute
        self.world_frame = 'base_link'
        self.robot_frame = 'trunk'
        if absolute:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.get_logger().info('Transform listener initialized for absolute rotation')
        
    def rotatezaxis_relative(self, angular_velocity):
        self.twist.angular.z = angular_velocity
        self.velocity_publisher.publish(self.twist)
        
    def euler_from_quaternion(self, q):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
        """
        x = q[0]
        y = q[1]
        z = q[2]
        w = q[3]

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        return roll, pitch, yaw
        
    def lookup_zrotation(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.robot_frame,
                self.world_frame,
                rclpy.time.Time())
        except TransformException as ex:
            self.get_logger().info(
                f'Could not transform {self.world_frame} to {self.robot_frame}: {ex}')
            return None
        q = np.empty((4, ))
        q[0] = t.transform.rotation.x
        q[1] = t.transform.rotation.y
        q[2] = t.transform.rotation.z
        q[3] = t.transform.rotation.w
        r, p, y = self.euler_from_quaternion(q)
        zrot = y / self.deg2rad  # Convert radians to degrees
        self.get_logger().info(f'go2 Z rotation: {zrot}')
        return y
          
    def handle_rotatezaxis(self, turn_angle, angular_velocity, start_angle, end_angle):
        response = { }
        response['atend'] = False
        response['message'] = 'robot successfully turned'
        self.get_logger().info('Requesting rotation: turn_angle {0} angular_velocity {1}'.format(turn_angle, angular_velocity))
        # Absolute rotation - ignore start_angle and end_angle
        # Compute amount to turn to get from current robot angle to turn_angle
        if angular_velocity <= 0:
            response['success'] = False
            response['message'] = 'error: angular_velocity must be greater than 0'
            self.get_logger().error(response['message'])
            return response
        if self.absolute:
            start_angle = self.lookup_zrotation()
            if start_angle is None:
                return {'success': False, 'message': 'Failed to get current angle from TF', 'last_angle': 0.0, 'elapsed_time': 0.0, 'atend': False}
            turn_angle = turn_angle - start_angle
        # Relative rotation - use start_angle and end_angle
        else:
            if turn_angle > 0:
                max_angle = end_angle - start_angle
                if max_angle < 0:
                    response['success'] = False
                    response['message'] = 'error: turn_angle is positive but end_angle is less than start_angle'
                    self.get_logger().error(response.message)
                    return response
                if max_angle < turn_angle:
                    turn_angle = max_angle
                    response['atend'] = True
                    response['message'] = 'robot at end angle'
            elif turn_angle < 0:
                min_angle = end_angle - start_angle
                if min_angle > 0:
                    response['success'] = False
                    response['message'] = 'error: turn_angle is negative but end_angle is greater than start_angle'
                    self.get_logger().error(response.message)
                    return response
                if min_angle > turn_angle:
                    turn_angle = min_angle
                    response['atend'] = True
                    response['message'] = 'robot at end angle'            
    
        duration = abs(turn_angle) / angular_velocity
        self.get_logger().info('Starting rotation: turn_angle {0} duration {1}'.format(turn_angle, duration))
        self.finish_event = Event()
        self.rotatezaxis_relative(angular_velocity)
        self.result_timer = self.create_timer(duration, self.publish_result, callback_group=self.callback_group)
        self.finish_event.wait()
        
        response['success'] = True
        response['last_angle'] = start_angle + turn_angle
        response['elapsed_time'] = duration
        response['message'] += ' last_angle {0} elapsed_time {1}'.format(response['last_angle'], duration)
        self.finish_event.clear()
        self.get_logger().info('Returning response: last_angle {0} elapsed_time {1}'.format(response['last_angle'], duration))
        return response
        
    def publish_result(self):
        self.rotatezaxis_relative(0.0)  # Stop the robot after rotation
        self.get_logger().info('Stopping rotation')
        self.result_timer.cancel()
        self.finish_event.set()
