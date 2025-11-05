
import unittest
import pytest
import rclpy
import numpy as np
from aidog_interfaces.srv import RotateZAxisRelative
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from launch.conditions import IfCondition

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_odom_arg = launch.actions.DeclareLaunchArgument(
            'mock_odom',
            default_value='true',
            description='Launch mock odom server'
        )
    launch_service = launch_ros.actions.Node(
            executable='aidog_rotatezaxis_relative_service',
            package='aidog_control',
            output='screen'
        )
    start_clock = launch_ros.actions.Node(
        executable='mock_odom_server',
        package='aidog_control',
        output='screen',
        condition=IfCondition(launch.substitutions.LaunchConfiguration('mock_odom'))
    )
    wait_for_service = launch.actions.TimerAction(
        period=2.0,
        actions=[launch_testing.actions.ReadyToTest()]
    )
    launch_desc = launch.LaunchDescription( [
        mock_odom_arg,
        start_clock,
        launch_service,
        wait_for_service,
    ] )
    return launch_desc
   
    
class TestRotateZAxisRelativeService(unittest.TestCase):
    check_duration = False
    @classmethod
    def setUpClass(self):
        # Initialize the ROS context for the test node
        rclpy.init()
        self.deg2rad = (np.pi / 180)
        self.rad2deg = (180 / np.pi)
        
    @classmethod
    def tearDownClass(self):
        # Shutdown the ROS context
        rclpy.shutdown()
                
    def setUp(self):
        self.node = rclpy.create_node('test_rotatezaxis_relative_service')
                 
    def tearDown(self):
        self.node.destroy_node()
    
    def create_client(self):
        """Create service client"""
        self.node.get_logger().info('Creating client')
        return self.node.create_client(RotateZAxisRelative, 'aidog_rotatezaxis_relative')
    
    def waitForService(self, client, timeout_sec=60.0):
        """Wait for service to be available"""
        self.node.get_logger().info('Waiting for service to be available')
        ready = client.wait_for_service(timeout_sec=timeout_sec)
        if not ready:
            raise RuntimeError('Wait for service timed out')
        self.node.get_logger().info('Service is available')

    def sendRequest(self, client, request):
        """Send request and wait for response"""
        self.node.get_logger().info('Sending request to service')
        future = client.call_async(request)
        self.assertIsNotNone(future)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            self.node.get_logger().info(future.result().message)
        else:
            self.node.get_logger().error('Service call failed.')
        return future.result()
            
    def atest_rotate_counterclockwise_60(self):
        """Test 60 degree rotation counterclockwise"""
        msgs_rx = []
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:
            self.waitForService(client)        
            request = RotateZAxisRelative.Request()
            request.turn_angle = 60.0 * self.deg2rad
            request.current_angle = 0.0
            request.angular_velocity = 10.0 * self.deg2rad
            request.end_angle = 180.0 * self.deg2rad
            duration = abs(request.turn_angle / request.angular_velocity)

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertFalse(response.at_end)
            self.assertEqual(round(response.last_angle * self.rad2deg), 60)
            if TestRotateZAxisRelativeService.check_duration:
                self.assertEqual(round(response.elapsed_time), duration)
            self.assertGreater(len(msgs_rx), 1)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
    
    def test_1rotate_clockwise_60(self):
        """Test 60 degree rotation clockwise"""
        msgs_rx = []
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:
            self.waitForService(client)        
            request = RotateZAxisRelative.Request()
            request.turn_angle = 60.0 * self.deg2rad
            request.current_angle = 0.0
            request.angular_velocity = -10.0 * self.deg2rad
            request.end_angle = 180.0 * self.deg2rad
            duration = abs(request.turn_angle / request.angular_velocity)
            response = self.sendRequest(client, request)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
            
            if response is not None:
                self.assertIsNotNone(response)
                self.assertTrue(response.success)
                self.assertFalse(response.at_end)
                self.assertTrue(np.isclose(round(response.last_angle * self.rad2deg), 300, 2))
                if TestRotateZAxisRelativeService.check_duration:
                    self.assertEqual(round(response.elapsed_time), duration)
                self.assertGreater(len(msgs_rx), 1)
            
    def atest_2rotate_30_badend(self):
        """Test 30 degree rotation with incorrect end_angle"""
        msgs_rx = []
        response = None
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:
            self.waitForService(client)        
            request = RotateZAxisRelative.Request()
            request.turn_angle = 30.0 * self.deg2rad
            request.current_angle = 0.0
            request.angular_velocity = 10.0 * self.deg2rad
            request.end_angle = -180.0 * self.deg2rad

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertFalse(response.success)
            self.assertIn('error', response.message)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
            
    def atest_3rotate60_atend(self):
        """Test 30 degree rotation with correct end_angle"""
        msgs_rx = []
        response = None
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')
      
        try:
            self.waitForService(client)        
            request = RotateZAxisRelative.Request()
            request.turn_angle = 60.0 * self.deg2rad
            request.current_angle = 30.0 * self.deg2rad
            request.angular_velocity = -10.0 * self.deg2rad
            request.end_angle = 80.0 * self.deg2rad

            response = self.sendRequest(client, request)

        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
        if response is not None:    
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertTrue(response.at_end)
            self.assertTrue(np.isclose(round(response.last_angle * self.rad2deg), 80, 2))
            if TestRotateZAxisRelativeService.check_duration:
                self.assertEqual(round(response.elapsed_time), 1)        
            self.assertIn('robot at end angle', response.message)

