import rclpy
import numpy as np
from pyquaternion import Quaternion
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from builtin_interfaces.msg import Time

class MockOdomServer(Node):
    def __init__(self):
        super().__init__('mock_odom_server')
        self.twist = None
        self.heading = 0.0
        self.xpos = 0.0
        self.ypos = 0.0   
        self.odom = Odometry()
        q = Quaternion(axis=[0, 0, 1], angle=0)
        self.odom.pose.pose.orientation.x = q[1]
        self.odom.pose.pose.orientation.y = q[2]
        self.odom.pose.pose.orientation.z = q[3]
        self.odom.pose.pose.orientation.w = q[0]
        header = Header()
        header.stamp = Time()
        header.frame_id = "odom" 
        self.odom.header = header
        self.prev_time = self.get_time()
        self.sub = self.create_subscription(
                Twist, 'cmd_vel',
                self.velocity_callback, 10)
        self.pub = self.create_publisher(Odometry, '/odom/raw', 1)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('Starting mock odometry server')
        
    def get_time(self):
        t = self.get_clock().now()
        seconds, nanos = t.seconds_nanoseconds()
        seconds += float(nanos) * 1e-9  # Convert nanoseconds to seconds
        return seconds
      
    def timer_callback(self):
        t = self.get_clock().now()
        seconds, nanos = t.seconds_nanoseconds()
        self.odom.header.stamp.sec = seconds
        self.odom.header.stamp.nanosec = nanos
        t = seconds + float(nanos) * 1e-9
        dt = t - self.prev_time
        self.prev_time = t
        if self.twist is not None:
            self.calc_odometry_pose(dt, self.odom)
        self.pub.publish(self.odom)
        
    def velocity_callback(self, msg):
        self.get_logger().info('Velocity command received: angular.z {:.2f}'.format(msg.angular.z))
        if (abs(msg.angular.z) + abs(msg.linear.x) + abs(msg.linear.y)) > 1e-6:
            self.twist = msg
        else:
            self.twist = None
        
    def calc_odometry_pose(self, dt, odom):
        delta_zangle = self.twist.angular.z * dt
        self.heading += delta_zangle
        if self.heading > np.pi * 2:
            self.heading -= np.pi * 2
        elif self.heading < -np.pi * 2:
            self.heading += np.pi * 2
        dx = (self.twist.linear.x * np.cos(self.heading) - self.twist.linear.y * np.sin(self.heading)) * dt
        dy = (self.twist.linear.x * np.sin(self.heading) + self.twist.linear.y * np.cos(self.heading)) * dt
        self.xpos += dx
        self.ypos += dy

        # robot's position in x,y, and z
        odom.pose.pose.position.x = self.xpos
        odom.pose.pose.position.y = self.ypos
        odom.pose.pose.position.z = 0.0
        
        # robot's heading_ in quaternion
        q = Quaternion(axis=[0, 0, 1], angle=self.heading)
        odom.pose.pose.orientation.x = q[1]
        odom.pose.pose.orientation.y = q[2]
        odom.pose.pose.orientation.z = q[3]
        odom.pose.pose.orientation.w = q[0]
        
        # covariance
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = 0.017
        odom.twist.covariance[0] = 0.3
        odom.twist.covariance[7] = 0.3
        odom.twist.covariance[35] = 0.017
        
        # velocity
        odom.twist.twist.linear.x = self.twist.linear.x
        odom.twist.twist.linear.y = self.twist.linear.y
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = self.twist.angular.z
        #self.get_logger().info('odom: dt={:.2f}, heading={:.2f} deg'.format(dt, self.heading * 180.0 / np.pi))

def main():
    rclpy.init()
    node = MockOdomServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()   
    