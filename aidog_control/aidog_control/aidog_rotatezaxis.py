
#!/usr/bin/env python3

from threading import Event
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import numpy as np
from pyquaternion import Quaternion
import rclpy
from rclpy.node import Node

    
class RotateZAxis(Node):
    def quaternion_angle(self, q0, q1):
        """
        Compute the angle (in radians) between two quaternions q0 and q1 (numpy arrays, x,y,z,w order).
        """
        q0_arr = np.array([q0.x, q0.y, q0.z, q0.w], dtype=float)
        q1_arr = np.array([q1.x, q1.y, q1.z, q1.w], dtype=float)

        q0_arr = q0_arr / np.linalg.norm(q0_arr)
        q1_arr = q1_arr / np.linalg.norm(q1_arr)
        dot = np.dot(q0_arr, q1_arr)
        # Clamp dot to valid range for arccos
        dot = np.clip(dot, -1.0, 1.0)
        angle = 2 * np.arccos(abs(dot))
        return angle
    
    def __init__(self, name, absolute=False):
        super().__init__(name)
        self.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 1)
        self.get_logger().info('Started RotateZAxis')
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.rad2deg = 180.0 / np.pi
        self.absolute = absolute

        self.duration = 0.0
        self.zrot = 0.0
        self.finish_event = Event()
        self.finish_event.clear()
        if absolute:
            self.starting_time = None
            self.pose_subscriber = self.create_subscription(Odometry, '/odom/raw',
                                                            self.pose_callback, 1,
                                                            callback_group=self.callback_group)
        else:
            self.starting_time = self.get_time()
            self.clock_subscriber = self.create_subscription(Clock, '/clock',
                                                            self.clock_callback, 1,
                                                            callback_group=self.callback_group)
    def stop(self):
        self.get_logger().info('Stopping RotateZAxis node')
        if self.velocity_publisher is not None:
            self.rotatezaxis_relative(0.0)  # Ensure to stop the robot
        self.node.destroy_publisher(self.velocity_publisher)
        self.velocity_publisher = None
        self.destroy_node()
        
    def get_time(self):
        t = self.get_clock().now()
        seconds, nanos = t.seconds_nanoseconds()
        seconds += float(nanos) * 1e-9  # Convert nanoseconds to seconds
        return seconds
        
    def pose_callback(self, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if self.starting_time == None:
            self.starting_time = t
        self.current_time = t
        if self.duration == 0:
            return
        ros_quat = msg.pose.pose.orientation
        q = Quaternion(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z)
        q = q.normalised
        axis = q.axis
        angle = q.angle
        prev_zrot = self.zrot
        self.zrot = angle

        #rx *= self.rad2deg  # Convert radians to degrees
        #ry *= self.rad2deg
        rz = angle * self.rad2deg
        dt = t - self.starting_time
        assert(np.allclose(axis, [0, 0, 1], atol=1e-3) or np.allclose(axis, [0, 0, -1], atol=1e-3)), f"Rotation axis is not Z: {axis}"
        #if abs(prev_zrot - self.zrot) > 0.001:  # Only log if significant change
        #    self.get_logger().info(f'aidog: axis={axis}, angle={angle} time: {dt}')
        if abs(self.last_angle - angle) < 1e-3:
            self.duration = 0
            self.publish_result()
                
    def clock_callback(self, msg):
        t = msg.clock
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        self.current_time = t
        if self.duration == 0:
            return
        dt = t - self.starting_time
        #self.get_logger().info(f'elapsed time: {dt}')
        if dt >= self.duration:
            self.duration = 0
            self.publish_result()

    def rotatezaxis_relative(self, angular_velocity):
        self.twist.angular.z = angular_velocity
        self.velocity_publisher.publish(self.twist)
        self.get_logger().info('Z angular velocity: {:.2f}'.format(angular_velocity)) 
          
    def handle_rotatezaxis(self, turn_angle, angular_velocity, start_angle, end_angle):
        self.response = { }
        self.response['success'] = False
        self.response['message'] = 'robot successfully turned'
        if turn_angle < 0:
            self.response['message'] = "error: turn angle " + str(turn_angle) + " must be positive"
            self.get_logger().error(self.response['message'])
            return self.response
        # Absolute rotation - ignore start_angle and end_angle
        # Compute amount to turn to get from current robot angle to turn_angle
        if angular_velocity == 0:
            self.response['message'] = 'error: angular_velocity cannot be 0'
            self.get_logger().error(self.response['message'])
            return self.response
        if self.absolute:
            self.handle_rotate_absolute(turn_angle, angular_velocity)
        else:
            self.handle_rotate_relative(turn_angle, angular_velocity, start_angle, end_angle)
        if 'error' in self.response['message']:
            return self.response
        self.response['elapsed_time'] = self.current_time - self.starting_time     
        self.response['message'] += ' last_angle {:.2f} elapsed_time {:.2f}'.format(self.response['last_angle'], self.response['elapsed_time'])
        self.get_logger().info('Returning response: ' + self.response['message'])
        return self.response
    
    def handle_rotate_relative(self, turn_angle, angular_velocity, start_angle, end_angle):
        self.response['at_end'] = False
        if start_angle < 0:
            self.response['message'] = "error: starting angle " + str(start_angle) + " must be positive"
            self.get_logger().error(self.response['message'])
            return self.response
        if end_angle < 0:
            self.response['message'] = "error: ending angle " + str(end_angle) + " must be positive"
            self.get_logger().error(self.response['message'])
            return self.response
        if end_angle < start_angle:
            self.response['message'] = "error: ending angle must be greater than starting angle"
            self.get_logger().error(self.response['message'])
            return self.response 
        self.get_logger().info('Requesting rotation: turn_angle {0} angular_velocity {1} start_angle {2}'.format(turn_angle, angular_velocity, start_angle))         
        if angular_velocity > 0:
            last_angle = start_angle + turn_angle
            if last_angle > end_angle:
                self.response['at_end'] = True
                self.response['message'] = 'robot at end angle'
                turn_angle = end_angle - start_angle
                last_angle = end_angle
        else:
            last_angle = start_angle - turn_angle
            if last_angle < 0:
                last_angle += (2 * np.pi)
            if last_angle < end_angle:
                self.response['at_end'] = True
                self.response['message'] = 'robot at end angle'
                turn_angle = end_angle - start_angle
                last_angle = end_angle

        duration = abs(turn_angle / angular_velocity)        
        self.get_logger().info('Starting rotation: turn_angle {0} duration {1}'.format(turn_angle, duration))
        self.response['last_angle'] = last_angle
        self.response['success'] = True
        self.starting_time = self.current_time
        if turn_angle != 0:
            self.rotatezaxis_relative(angular_velocity)
            self.duration = duration
            self.finish_event.wait()
            self.finish_event.clear()
    
    def handle_rotate_absolute(self, turn_angle, angular_velocity):
        self.last_angle = turn_angle
        amount_to_turn = self.zrot - turn_angle
        flip_last_angle = False
        if (angular_velocity < 0) and (turn_angle > self.zrot):  # turn through 0:
            self.last_angle = 2 * np.pi - turn_angle
            flip_last_angle = True
            amount_to_turn = self.zrot + self.last_angle
        if (angular_velocity > 0) and (turn_angle < self.zrot):  # turn through 0:
            amount_to_turn = (2 * np.pi - self.zrot) + turn_angle                     
        duration = abs(amount_to_turn / angular_velocity)       
        self.get_logger().info('Starting rotation: turn_angle {:.2f} angular_velocity {:.2f} duration {:.2f} last_angle {:2f}'.format(turn_angle, angular_velocity, duration, self.last_angle))

        self.response['success'] = True
        self.starting_time = self.current_time

        if not np.isclose(turn_angle, self.zrot, atol=1e-3):
            self.rotatezaxis_relative(angular_velocity)
            self.duration = duration
            self.finish_event.wait()
            self.finish_event.clear()       
        if flip_last_angle:
            self.response['last_angle'] = 2 * np.pi - self.zrot
        else:
            self.response['last_angle'] = self.zrot                      
        
    def publish_result(self):
        self.rotatezaxis_relative(0.0)  # Stop the robot after rotation
        self.get_logger().info('Stopping rotation')
        self.finish_event.set()

    def slerp_quaternion(self, q0, q1, t):
        """
        Spherical linear interpolation between two quaternions q0 and q1.
        q0, q1: numpy arrays of shape (4,) in (x, y, z, w) order
        t: interpolation parameter between 0 and 1
        Returns: interpolated quaternion as numpy array (x, y, z, w)
        """
        # Normalize input quaternions
        q0 = q0 / np.linalg.norm(q0)
        q1 = q1 / np.linalg.norm(q1)
        dot = np.dot(q0, q1)
        # If the dot product is negative, slerp won't take the shorter path.
        # Fix by reversing one quaternion.
        if dot < 0.0:
            q1 = -q1
            dot = -dot
        DOT_THRESHOLD = 0.9995
        if dot > DOT_THRESHOLD:
            # If the quaternions are close, use linear interpolation
            result = q0 + t * (q1 - q0)
            return result / np.linalg.norm(result)
        # Compute the angle between the quaternions
        theta_0 = np.arccos(dot)
        sin_theta_0 = np.sin(theta_0)
        theta = theta_0 * t
        sin_theta = np.sin(theta)
        s0 = np.cos(theta) - dot * sin_theta / sin_theta_0
        s1 = sin_theta / sin_theta_0
        return (s0 * q0) + (s1 * q1)