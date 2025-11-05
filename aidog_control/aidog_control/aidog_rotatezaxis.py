
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
        self.twist = Twist()
        self.twist.linear.x = 0.0
        self.twist.linear.y = 0.0
        self.twist.linear.z = 0.0
        self.twist.angular.x = 0.0
        self.twist.angular.y = 0.0
        self.twist.angular.z = 0.0
        self.rad2deg = 180.0 / np.pi
        self.absolute = absolute
        self.current_time = 0.0
        self.starting_time = 0.0
        self.duration = 0.0
        self.zrot = 0.0
        self.finish_event = Event()
        self.finish_event.clear()
        self.first_angle = None
        self.pose_subscriber = self.create_subscription(Odometry, '/odom/raw',
                                                        self.pose_callback, 1)
        self.get_logger().info('Subscribing to /odom/raw')
            
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
    
    def positive_angle(self, angle):
        if angle < 0:
            angle += 2 * np.pi
        return np.fmod(angle, 2 * np.pi)
        
    def positive_angle_deg(self, angle):
        if angle < 0:
            angle += 360.0
        return np.fmod(angle, 360.0)
    
    def pose_callback(self, msg):
        self.current_time = t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9 
        ros_quat = msg.pose.pose.orientation
        q = Quaternion(ros_quat.w, ros_quat.x, ros_quat.y, ros_quat.z)
        q = q.normalised
        axis = q.axis
        orig_angle = q.angle
        angle = q.angle
        prev_zrot = self.zrot
        zaxis = np.allclose(axis, [0, 0, 1], atol=1e-3)
        negzaxis = np.allclose(axis, [0, 0, -1], atol=1e-3)
        if axis[2] < 0:      # rotate about -Z axis?
            angle = -angle
        self.zrot = angle = self.positive_angle(angle)
        if self.duration == 0:
            return
        dt = t - self.starting_time 
        s = 'axis={0}'.format(axis)
        angle_deg = angle * self.rad2deg
        last_angle_deg = self.last_angle * self.rad2deg
        s += ', angle={:.2f}'.format(orig_angle * self.rad2deg)         
        if not zaxis and not negzaxis:   # rotate about Z axis?
            self.get_logger().error(f"rotation not about Z axis, axis =  {axis}")
            return           
        s += ', dt={:.2f}, new angle={:.2f}'.format(dt, angle_deg)
        self.get_logger().info(s)
        finished = abs(angle_deg - last_angle_deg ) < 1
        finished = finished or (prev_zrot > angle) and (prev_zrot > self.last_angle) and (angle <= self.last_angle)
        finished = finished or (prev_zrot < angle) and (prev_zrot < self.last_angle) and (angle >= self.last_angle)
        if finished:
            self.duration = 0
            self.finish_angle = angle
            self.get_logger().info("last angle: requested = {:.2f}, actual = {:.2f}".format(self.last_angle * self.rad2deg, angle * self.rad2deg))
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
    
    def handle_rotate_relative_old(self, turn_angle, angular_velocity, start_angle, end_angle):
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
            if last_angle < end_angle:
                self.response['at_end'] = True
                self.response['message'] = 'robot at end angle'
                turn_angle = end_angle - start_angle
                last_angle = end_angle
            last_angle = self.positive_angle(last_angle)

        duration = abs(turn_angle / angular_velocity)        
        self.response['last_angle'] = last_angle
        self.response['success'] = True
        self.starting_time = self.current_time
        if turn_angle != 0:
            self.get_logger().info('Starting rotation: turn_angle {0} duration {1}'.format(turn_angle, duration))
            self.rotatezaxis_relative(angular_velocity)
            self.duration = duration
            self.finish_event.wait()
            self.finish_event.clear()
    
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
        self.get_logger().info('Requesting rotation: turn_angle {:.2f} angular_velocity {:.2f} start_angle {:.2f}'.format(turn_angle, angular_velocity, start_angle))         
        current_angle = self.zrot
        self.finish_angle = current_angle
        if turn_angle > (end_angle - start_angle):
            self.response['at_end'] = True
            self.response['message'] = 'robot at end angle'
            turn_angle = abs(end_angle - start_angle)
        if angular_velocity > 0:
            if current_angle < 1e-3:
                current_angle = 2 * np.pi
            self.last_angle = current_angle + turn_angle 
        else:       
            self.last_angle = current_angle - turn_angle       
            self.last_angle = self.positive_angle(self.last_angle)
        self.get_logger().info('Before rotation: current angle {:.2f} last angle {:.2f}'.format(current_angle * self.rad2deg, self.last_angle * self.rad2deg))        
        duration = abs(turn_angle / angular_velocity)        
        self.response['success'] = True
        self.starting_time = self.current_time

        if turn_angle != 0:
            self.get_logger().info('Starting rotation: turn_angle {:.2f} duration {:.2f}'.format(turn_angle, duration))
            self.rotatezaxis_relative(angular_velocity)
            self.duration = duration
            self.finish_event.wait()
            self.finish_event.clear()
        if angular_velocity > 0:
            turn_amount = self.finish_angle - current_angle
            self.response['last_angle'] = start_angle + turn_amount
        else:
            turn_amount = current_angle - self.finish_angle
            self.response['last_angle'] = self.positive_angle(start_angle - turn_amount)
        self.get_logger().info('turn amount requested: {:.2f}, actual {:.2f}'.format(turn_angle, turn_amount))
    
    def handle_rotate_absolute(self, turn_angle, angular_velocity):
        self.last_angle = turn_angle
        amount_to_turn = self.zrot - turn_angle
        flip_last_angle = False
        if (angular_velocity < 0) and (turn_angle > self.zrot):  # turn through 0:
            self.last_angle = 2 * np.pi - turn_angle
            #flip_last_angle = True
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