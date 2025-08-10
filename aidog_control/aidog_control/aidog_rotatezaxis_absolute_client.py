#!/usr/bin/env python3


from aidog_interfaces.srv import RotateZAxisAbsolute

import rclpy

from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import sys


class RotateZAxisAbsoluteClient(Node):

    def __init__(self, turn_angle=30.0, angular_velocity=0.1):
        super().__init__('rotatezaxis_absolute_client')
        self.get_logger().info('Started RotateZAxisAbsoluteClient node')
        self.cli = self.create_client(RotateZAxisAbsolute, 'aidog_rotatezaxis_absolute')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = RotateZAxisAbsolute.Request()
        self.req.turn_angle = turn_angle
        self.req.angular_velocity = angular_velocity

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
        deg2rad = (3.14159 / 180)
        angular_velocity = 5.0  # degrees per second
        # Parse command-line arguments if provided
        if len(sys.argv) > 1:
            turn_angle = float(sys.argv[1])                         
        if len(sys.argv) > 2:
            angular_velocity = float(sys.argv[2])
        turn_angle = turn_angle * deg2rad # Convert degrees to radians
        angular_velocity = angular_velocity * deg2rad  # Convert degrees per second to radians per second
        client = RotateZAxisAbsoluteClient(turn_angle, angular_velocity)
        client.send_request()

if __name__ == '__main__':
    main()