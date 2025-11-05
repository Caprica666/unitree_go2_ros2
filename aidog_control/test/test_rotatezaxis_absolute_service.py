
import unittest
import pytest
import rclpy
import numpy as np
from aidog_interfaces.srv import RotateZAxisAbsolute
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
            description='Launch mock odometry server'
        )
    launch_service = launch_ros.actions.Node(
            executable='aidog_rotatezaxis_absolute_service',
            package='aidog_control',
            output='screen'
        )
    start_odom = launch_ros.actions.Node(
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
        start_odom,
        launch_service,
        wait_for_service,
    ] )
    return launch_desc
   
    
class TestRotateZAxisAbsoluteService(unittest.TestCase):
    current_angle = 0.0
    
    @classmethod
    def setUpClass(self):
        # Initialize the ROS context for the test node
        rclpy.init()
        self.deg2rad = np.pi / 180
        self.rad2deg = 180 / np.pi

        
    @classmethod
    def tearDownClass(self):
        # Shutdown the ROS context
        rclpy.shutdown()
                
    def setUp(self):
        self.node = rclpy.create_node('test_rotatezaxis_absolute_service')
                
    def tearDown(self):
        self.node.destroy_node()
    
    def create_client(self):
        """Create service client"""
        self.node.get_logger().info('Creating client')
        return self.node.create_client(RotateZAxisAbsolute, 'aidog_rotatezaxis_absolute')
    
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
    
    def rotateTo(self, angle, angular_velocity, num_msgs=0):
        msgs_rx = []
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)

        try:
            self.waitForService(client)        
            request = RotateZAxisAbsolute.Request()
            request.turn_angle = angle * self.deg2rad
            request.angular_velocity = angular_velocity * self.deg2rad
            amount_to_turn = TestRotateZAxisAbsoluteService.current_angle - angle
            if (angular_velocity < 0) and (angle > TestRotateZAxisAbsoluteService.current_angle):  # turn through 0:
                amount_to_turn = (360 - angle) + TestRotateZAxisAbsoluteService.current_angle
            if (angular_velocity > 0) and (angle < TestRotateZAxisAbsoluteService.current_angle):  # turn through 0:
                amount_to_turn = (360 - TestRotateZAxisAbsoluteService.current_angle) + angle  
            duration = round(abs(amount_to_turn / angular_velocity))
            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertEqual(round(response.last_angle * self.rad2deg), round(angle))
            self.assertEqual(round(response.elapsed_time), duration)
            self.assertGreaterEqual(len(msgs_rx), num_msgs)
            TestRotateZAxisAbsoluteService.current_angle = response.last_angle * self.rad2deg

        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
        
    def test_1rotateto0(self):
        """Test rotation to zero degrees clockwise"""
        angle = 0
        angular_velocity = -10
        client = self.create_client()

        try:
            self.waitForService(client)        
            request = RotateZAxisAbsolute.Request()
            request.turn_angle = angle * self.deg2rad
            request.angular_velocity = angular_velocity * self.deg2rad
            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertEqual(round(response.last_angle * self.rad2deg), round(angle))
            TestRotateZAxisAbsoluteService.current_angle = response.last_angle * self.rad2deg

        finally:
            self.node.destroy_client(client)
            
    def test_2rotateto30(self):
        """Test rotate to 30 clockwise"""
        self.rotateTo(30, 10, 1)
        
    def atest_3rotateto60(self):
        """Test rotate to 60 clockwise"""
        self.rotateTo(60, 10, 1)
        
    def atest_4rotateto30_counter(self):
        """Test rotate to 30 counterclockwise"""
        self.rotateTo(30, -10, 1)
    
    def atest_5rotateto330_counter(self):
        """Test rotate to 330 counterclockwise"""
        self.rotateTo(330, -10, 1)
        
    def atest_6rotateto300_counter(self):
        """Test rotate to 300 counterclockwise"""
        self.rotateTo(300, -10, 1)
        
    def atest_7rotateto0(self):
        """Test rotate to 0 clockwise"""
        self.rotateTo( 0, 10, 0)
        

