import unittest
import pytest
import rclpy
from aidog_interfaces.action import RotateZAxisRelative
from rclpy.action import ActionClient
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from launch.conditions import IfCondition

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_clock_arg = launch.actions.DeclareLaunchArgument(
            'mock_clock',
            default_value='true',
            description='Launch mock clock server'
        )
    
    launch_action = launch_ros.actions.Node(
            executable='aidog_rotatezaxis_relative_action',
            package='aidog_control',
            output='screen'
        )
    start_clock = launch_ros.actions.Node(
        executable='clock_server',
        package='aidog_control',
        output='screen',
        condition=IfCondition(launch.substitutions.LaunchConfiguration('mock_clock'))
    )
    wait_for_launch = launch.actions.TimerAction(
        period=2.0,
        actions=[launch_testing.actions.ReadyToTest()]
    )
    launch_desc = launch.LaunchDescription(
            [
                mock_clock_arg,
                start_clock,
                launch_action,
                wait_for_launch,
            ]
        )
       
    return launch_desc
   
    
class TestRotateZAxisRelativeAction(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # Initialize the ROS context for the test node
        rclpy.init()
        self.deg2rad = (3.14159 / 180)
        self.rad2deg = (180 / 3.14159)
        
    @classmethod
    def tearDownClass(self):
        # Shutdown the ROS context
        rclpy.shutdown()
                
    def setUp(self):
        self.node = rclpy.create_node('test_rotatezaxis_relative_action')
                
    def tearDown(self):
        self.node.destroy_node()
    
    def create_client(self):
        """Create action client"""
        self.node.get_logger().info('Creating client')
        return ActionClient(self.node, RotateZAxisRelative, 'aidog_rotatezaxis_relative_action_server')
    
    def waitForService(self, client, timeout_sec=2.0):
        """Wait for service to be available"""
        self.node.get_logger().info('Waiting for server to be available')
        ready = client.wait_for_server(timeout_sec=timeout_sec)
        if not ready:
            raise RuntimeError('Wait for server timed out')
        self.node.get_logger().info('Server is available')

    def sendRequest(self, client, goal_msg):
        self.node.get_logger().info('Sending goal with turn_angle: {0}, start_angle: {1}, angular_velocity: {2} end_angle: {3}'.format(
            goal_msg.turn_angle, goal_msg.start_angle, goal_msg.angular_velocity, goal_msg.end_angle))
        self._send_goal_future = client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)
        self.assertIsNotNone(self._send_goal_future)
        return self.waitForResult(self._send_goal_future)
        
    def waitForResult(self, send_goal_future):
        rclpy.spin_until_future_complete(self.node, send_goal_future)
        goal_handle = send_goal_future.result()
        if goal_handle.accepted:
            self.get_result_future = goal_handle.get_result_async()
            self.get_result_future.add_done_callback(self.get_result_callback)
        if self.get_result_future is not None:
            rclpy.spin_until_future_complete(self.node, self.get_result_future)
            result = self.get_result_future.result().result
            self.node.get_logger().info('Result {0}'.format(result));
            self.node.get_logger().info(result.message)
            return result
        else:
            self.node.get_logger().error('Action call failed.')
        return None

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.node.get_logger().info('Goal rejected :(')
        else:
            self.node.get_logger().info('Goal accepted :)')

    def get_result_callback(self, future):
        result = future.result().result
        self.node.get_logger().info('Result: message = {0}'.format(result.message))

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #self.node.get_logger().info('Received feedback: current_angle {0} current_time {1}'.format(feedback.current_angle, feedback.current_time))
            
    def test_rotate30(self):
        """Test 30 degree rotation"""
        msgs_rx = []
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:
            self.waitForService(client)
            request = RotateZAxisRelative.Goal()      
            request.turn_angle = 30.0 * self.deg2rad
            request.angular_velocity = 10.0 * self.deg2rad
            request.end_angle = 180.0 * self.deg2rad
            request.start_angle = 0.0
            duration = abs(request.turn_angle / request.angular_velocity)

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertFalse(response.at_end)
            self.assertEqual(round(response.last_angle * self.rad2deg), 30)
            self.assertEqual(round(response.elapsed_time), duration)
            self.assertGreater(len(msgs_rx), 1)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
    
    def test_rotateneg30(self):
        """Test -30 degree rotation"""
        msgs_rx = []
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:
            self.waitForService(client)        
            request = RotateZAxisRelative.Goal()
            request.turn_angle = -30.0 * self.deg2rad
            request.start_angle = 0.0
            request.angular_velocity = 10.0 * self.deg2rad
            request.end_angle = -180.0 * self.deg2rad
            duration = abs(request.turn_angle / request.angular_velocity)

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertFalse(response.at_end)
            self.assertEqual(round(response.last_angle * self.rad2deg), -30)
            self.assertEqual(round(response.elapsed_time), duration)
            self.assertGreater(len(msgs_rx), 1)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
            
    def test_rotateneg30fail(self):
        """Test -30 degree rotation with incorrect end_angle"""
        msgs_rx = []
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:
            self.waitForService(client)        
            request = RotateZAxisRelative.Goal()
            request.turn_angle = -30.0 * self.deg2rad
            request.start_angle = 0.0
            request.angular_velocity = 10.0 * self.deg2rad
            request.end_angle = 180.0 * self.deg2rad

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertFalse(response.success)
            self.assertIn('error', response.message)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)
            
    def test_turn_robot_camera_end_angle_neg(self):
        """Test -30 degree rotation with correct end_angle"""
        msgs_rx = []
        client = self.create_client()
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')
      
        try:
            self.waitForService(client)        
            request = RotateZAxisRelative.Goal()
            request.turn_angle = -30.0 * self.deg2rad
            request.start_angle = -60.0 * self.deg2rad
            request.angular_velocity = 10.0 * self.deg2rad
            request.end_angle = -70.0 * self.deg2rad

            
            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertTrue(response.at_end)
            self.assertEqual(round(response.last_angle * self.rad2deg), -70)
            self.assertEqual(round(response.elapsed_time), 1)
            self.assertIn('robot at end angle', response.message)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)

