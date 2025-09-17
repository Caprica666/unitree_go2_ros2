#!/home/nolad/projects/robotics/dog_ws/.venv/bin/python3
import unittest
import pytest
import rclpy
import json
import asyncio
import aiohttp

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
    start_clock = launch_ros.actions.Node(
        executable='clock_server',
        package='aidog_control',
        output='screen',
        condition=IfCondition(launch.substitutions.LaunchConfiguration('mock_clock'))
    )
    launch_service = launch_ros.actions.Node(
            executable='aidog_web_rotatezaxis_relative',
            package='aidog_control',
            output='screen'
        )
    web_server = launch.actions.ExecuteProcess(
        cmd=[
            "ros2",
            "web",
            "server",
            '--no-auth',
            '--port=5000',
            '--log-level=INFO',
            '--timeout=20'
            ],
        output='screen'
    )
    wait_for_service = launch.actions.TimerAction(
        period=2.0,
        actions=[launch_testing.actions.ReadyToTest()]
    )
    launch_desc = launch.LaunchDescription(
            [
                mock_clock_arg,
                web_server,
                start_clock,
                launch_service,
                wait_for_service,
            ]
        )
    return launch_desc
   
    
class TestWebRotateZAxisRelativeService(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # Initialize the ROS context for the test node
        rclpy.init()
        self.timeout = 20.0
        self.robot_url = "http://localhost:5000/aidog_rotatezaxis_relative"
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop) 
        
    @classmethod
    def tearDownClass(self):
        # Shutdown the ROS context
        rclpy.shutdown()
                
    def setUp(self):
        self.node = rclpy.create_node('test_web_rotatezaxis_relative_service')
                
    def tearDown(self):
        self.node.destroy_node()

    async def sendGetRequest(self, request):
        async with aiohttp.ClientSession() as session:
            async with session.get(self.robot_url, params = request, timeout=self.timeout) as response:
                return await response.json()
    
    async def sendPostRequest(self, request):
        """Send request and wait for response"""
        async with aiohttp.ClientSession() as session:
            async with session.post(self.robot_url, json = request, timeout=self.timeout) as response:
                return await response.json()
            
    async def sendFormRequest(self, request):
        """Send request and wait for response"""
        async with aiohttp.ClientSession() as session:
            form = aiohttp.FormData()
            for key, val in request.items():
                form.add_field(key, str(val))
            async with session.post(self.robot_url, data = form,
                                    headers={"Content-Type": "application/x-www-form-urlencoded"},
                                    timeout=self.timeout) as response:
                return await response.json()
    
    async def parseResponse(self, response):
        """Parse response and return dictionary"""
        response_dict = { "success": False }
        try:
            if response.status == 200:
                if response.headers.get('Content-Type') == 'application/json':
                    response_dict = response.json()
                    self.node.get_logger().error(f"Request returned: {response_dict}")
                elif response.headers.get('Content-Type') == 'text/plain':
                    # If the response is plain text, parse it as JSON
                    response_dict = json.loads(response.text())
                    self.node.get_logger().error(f"Request returned: {response_dict}")
                else:
                    response_dict["message"] = "error: aidog_rotatezaxis_relative Unexpected content type"
                    self.node.get_logger().error(f"Unexpected content type: {response.text()}")
                    return response_dict
            else:
                if response.headers.get('Content-Type') == 'application/json':
                    response_dict = response.json()
                    if "message" not in response_dict:
                        response_dict["message"] = "error: aidog_rotatezaxis_relative JSONrequest failed " + str(response.text)
                else:
                    response_dict["message"] = "error: aidog_rotatezaxis_relative request failed " + str(response.text)
                self.node.get_logger().error(response_dict["message"])               
        except Exception as e:
            self.node.get_logger().error(f"Request failed: {e}")
            response_dict["message"] = "error: aidog_rotatezaxis_relative " + str(e)
        return response_dict
            
    def test_get_rotate30(self):
        """Test 30 degree rotation using get"""
        msgs_rx = []
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)

        try:    
            request = { }
            request['turn_angle'] = 30.0
            request['current_angle'] = 0.0
            request['angular_velocity'] = 10.0
            request['end_angle'] = 180.0
            duration = abs(request['turn_angle'] / request['angular_velocity'])

            response = asyncio.run(self.sendGetRequest(request))
            self.node.get_logger().error(f"Response returned: {response}")
            self.assertIsNotNone(response)
            self.assertTrue(response['success'])
            self.assertFalse(response['at_end'])
            self.assertEqual(round(response['last_angle']), 30)
            self.assertEqual(round(response['elapsed_time']), duration)
        finally:
            self.node.destroy_subscription(sub)
            
    def test_post_rotate30(self):
        """Test 30 degree rotation using json post"""
        msgs_rx = []
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)

        try:    
            request = { }
            request['turn_angle'] = 30.0
            request['current_angle'] = 0.0
            request['angular_velocity'] = 10.0
            request['end_angle'] = 180.0
            duration = abs(request['turn_angle'] / request['angular_velocity'])

            response = asyncio.run(self.sendPostRequest(request))
            self.node.get_logger().error(f"Response returned: {response}")
            self.assertIsNotNone(response)
            self.assertTrue(response['success'])
            self.assertFalse(response['at_end'])
            self.assertEqual(round(response['last_angle']), 30)
            self.assertEqual(round(response['elapsed_time']), duration)
        finally:
            self.node.destroy_subscription(sub)
        
    def test_form_rotate30(self):
        """Test 30 degree rotation using form post"""
        msgs_rx = []
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)

        try:    
            request = { }
            request['turn_angle'] = 30.0
            request['current_angle'] = 0.0
            request['angular_velocity'] = 10.0
            request['end_angle'] = 180.0
            duration = abs(request['turn_angle'] / request['angular_velocity'])

            response = asyncio.run(self.sendFormRequest(request))
            self.node.get_logger().error(f"Response returned: {response}")
            self.assertIsNotNone(response)
            self.assertTrue(response['success'])
            self.assertFalse(response['at_end'])
            self.assertEqual(round(response['last_angle']), 30)
            self.assertEqual(round(response['elapsed_time']), duration)
        finally:
            self.node.destroy_subscription(sub)
    
    def atest_get_rotateneg30(self):
        """Test -30 degree rotation"""
        msgs_rx = []
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:      
            request = { }
            request['turn_angle'] = -30.0
            request['current_angle'] = 0.0
            request['angular_velocity'] = 10.0
            request['end_angle'] = -180.0
            duration = abs(request['turn_angle'] / request['angular_velocity'])

            response = asyncio.run(self.sendGetRequest(request))
            self.assertIsNotNone(response)
            self.assertTrue(response['success'])
            self.assertFalse(response['at_end'])
            self.assertEqual(round(response['last_angle']), -30)
            self.assertEqual(round(response['elapsed_time']), duration)
            self.assertGreater(len(msgs_rx), 1)
        finally:
            self.node.destroy_subscription(sub)
            
    def test_get_rotateneg30fail(self):
        """Test -30 degree rotation with incorrect end_angle"""
        msgs_rx = []
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')

        try:
            request = { }
            request['turn_angle'] = -30.0
            request['current_angle'] = 0.0
            request['angular_velocity'] = 10.0
            request['end_angle'] = 180.0

            response = asyncio.run(self.sendGetRequest(request))
            self.assertIsNotNone(response)
            self.assertFalse(response['success'])
            self.assertIn('error', response['message'])
        finally:
            self.node.destroy_subscription(sub)
            
    def test_get_turn_robot_camera_end_angle_neg(self):
        """Test -30 degree rotation with correct end_angle"""
        msgs_rx = []
        sub = self.node.create_subscription(
                Twist, 'cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        self.node.get_logger().info('Subscribing to cmd_vel topic')
      
        try:
            request = { }
            request['turn_angle'] = -30.0
            request['current_angle'] = -60.0
            request['angular_velocity'] = 10.0
            request['end_angle'] = -70.0
            duration = abs(-10 / request['angular_velocity'])

            response = asyncio.run(self.sendGetRequest(request))
            self.assertIsNotNone(response)
            self.assertTrue(response['success'])
            self.assertTrue(response['at_end'])
            self.assertEqual(round(response['last_angle']), -70)
            self.assertEqual(round(response['elapsed_time']), duration)
            self.assertIn('robot at end angle', response['message'])
        finally:
            self.node.destroy_subscription(sub)

