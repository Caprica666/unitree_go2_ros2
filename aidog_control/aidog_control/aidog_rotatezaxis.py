
#!/usr/bin/env python3

from threading import Event
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import numpy as np
import quaternion
import rclpy
from rclpy.node import Node
from rclpy.time import Time

from tf2_ros.transform_listener import TransformListener
    
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
        self.starting_time = self.get_time()
        self.duration = 0.0
        self.zrot = 0.0
        self.finish_event = Event()
        self.finish_event.clear()
        if absolute:
            self.pose_subscriber = self.create_subscription(Odometry, '/odom/raw',
                                                            self.pose_callback, 1,
                                                            callback_group=self.callback_group)
        else:
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
    
    def quat_to_axis_angle(self, q):
        """
        Converts quaternion (w in last place) to axis-angle representation.
        quaternion = [x, y, z, w]
        Returns: (axis, angle) where axis is a numpy array and angle is in radians.
        """
        angle = 2 * np.arccos(q.w)
        s = np.sqrt(1 - q.w * q.w)
        if s < 1e-6:
            return np.array([1.0, 0.0, 0.0]), angle
        axis = np.array([q.x, q.y, q.z]) / s
        return axis, angle
        
    def pose_callback(self, msg):
        q = msg.pose.pose.orientation
        q = quaternion.quaternion(q.x, q.y, q.z, q.w)
        q = q.normalized()
        rx, ry, rz = self.euler_from_quaternion(q.x, q.y, q.z, q.w)
        prev_zrot = self.zrot
        self.zrot = rz
        if self.duration == 0:
            return
        rx = rx / self.deg2rad  # Convert radians to degrees
        ry = ry / self.deg2rad
        rz = rz / self.deg2rad
        #if abs(prev_zrot - self.zrot) > 0.001:  # Only log if significant change
        #    self.get_logger().info(f'angle: {rz}')
        dt = self.get_time() - self.starting_time
        if dt >= self.duration:
            self.publish_result()
                
    def clock_callback(self, msg):
        if self.duration == 0:
            return
        t = msg.clock
        t = msg.clock.sec + msg.clock.nanosec * 1e-9
        dt = t - self.starting_time
        #self.get_logger().info(f'elapsed time: {dt}')
        if dt >= self.duration:
            self.publish_result()

    def rotatezaxis_relative(self, angular_velocity):
        self.twist.angular.z = angular_velocity
        self.get_logger().info(f'Z angular velocity: {angular_velocity}')
        self.velocity_publisher.publish(self.twist)
        
    def euler_from_quaternion(self, x, y, z, w):
        """
        Converts quaternion (w in last place) to euler roll, pitch, yaw
        quaternion = [x, y, z, w]
        """
        q = quaternion.quaternion(x, y, z, w)
        q = q.normalized()
        arr = quaternion.as_euler_angles(q)  # Convert quaternion to euler angles
        return arr[0], arr[1], arr[2]
          
    def handle_rotatezaxis(self, turn_angle, angular_velocity, start_angle, end_angle):
        self.response = { }
        self.response['at_end'] = False
        self.response['success'] = False
        self.response['message'] = 'robot successfully turned'
        # Absolute rotation - ignore start_angle and end_angle
        # Compute amount to turn to get from current robot angle to turn_angle
        if angular_velocity <= 0:
            self.response['message'] = 'error: angular_velocity must be greater than 0'
            self.get_logger().error(self.response['message'])
            return self.response
        if self.absolute:
            start_angle = self.zrot
            turn_angle = turn_angle - start_angle
            self.get_logger().info('Requesting rotation: turn_angle {0} angular_velocity {1} start_angle {2}'.format(turn_angle, angular_velocity, start_angle))

        # Relative rotation - use start_angle and end_angle
        else:
            if turn_angle > 0:
                max_angle = end_angle - start_angle
                if max_angle < 0:
                    self.response['message'] = "error: turn_angle " + str(turn_angle) + " is positive but end_angle " + str(end_angle) + " is less than start_angle " + str(start_angle)
                    self.get_logger().error(self.response['message'])
                    return self.response
                if max_angle < turn_angle:
                    turn_angle = max_angle
                    self.response['at_end'] = True
                    self.response['message'] = 'robot at end angle'
            elif turn_angle < 0:
                min_angle = end_angle - start_angle
                if min_angle > 0:
                    self.response['message'] = "error: turn_angle " + str(turn_angle) + " is negative but end_angle " + str(end_angle) + " is greater than start_angle " + str(start_angle)
                    self.get_logger().error(self.response['message'])
                    return self.response
                if min_angle > turn_angle:
                    turn_angle = min_angle
                    self.response['at_end'] = True
                    self.response['message'] = 'robot at end angle' 
            self.get_logger().info('Requesting rotation: turn_angle {0} angular_velocity {1}'.format(turn_angle, angular_velocity))
        duration = abs(turn_angle) / angular_velocity         
        self.get_logger().info('Starting rotation: turn_angle {0} duration {1}'.format(turn_angle, duration))
        self.response['last_angle'] = start_angle + turn_angle
        self.response['elapsed_time'] = duration
        self.response['message'] += ' last_angle {0} elapsed_time {1}'.format(self.response['last_angle'], duration)
        self.response['success'] = True
        if turn_angle != 0:
            self.rotatezaxis_relative(angular_velocity)
            self.starting_time = self.get_time()
            self.duration = duration
            self.finish_event.wait()
            self.finish_event.clear()       
        if self.absolute:
            self.get_logger().info('Absolute rotation completed: z_rot {0}'.format(self.zrot))
        self.get_logger().info('Returning response: last_angle {0} elapsed_time {1}'.format(self.response['last_angle'], duration))
        return self.response
        
    def publish_result(self):
        self.duration = 0
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