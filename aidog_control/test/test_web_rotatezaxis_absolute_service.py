#!/home/nolad/projects/robotics/dog_ws/.venv/bin/python3
import threading
import unittest
import pytest
import rclpy
import json
import asyncio
import aiohttp
import time

from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from launch.conditions import IfCondition
from aidog_interfaces.srv import RotateZAxisAbsolute

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    mock_odom_arg = launch.actions.DeclareLaunchArgument(
            'mock_odom',
            default_value='true',
            description='Launch mock odometry server'
        )
    start_clock = launch_ros.actions.Node(
        executable='mock_odom_server',
        package='aidog_control',
        output='screen',
        condition=IfCondition(launch.substitutions.LaunchConfiguration('mock_odom'))
    )
    launch_service = launch_ros.actions.Node(
            executable='aidog_web_rotatezaxis_absolute',
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
                mock_odom_arg,
                web_server,
                start_clock,
                launch_service,
                wait_for_service,
            ]
        )
    return launch_desc
   
    
class TestWebRotateZAxisAbsoluteService(unittest.TestCase):
    current_angle = 0.0
    
    @classmethod
    def setUpClass(self):
        # Initialize the ROS context for the test node
        rclpy.init()
        self.timeout = 20.0
        self.robot_url = "http://localhost:5000/aidog_rotatezaxis_absolute"
        self.event_loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.event_loop)
        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
        self.spin_thread.start() 
        
    @classmethod
    def tearDownClass(self):
        # Shutdown the ROS context
        self.executor.shutdown()
        rclpy.shutdown()
                
    def setUp(self):
        self.node = rclpy.create_node('test_web_rotatezaxis_absolute_service')
        self.executor.add_node(self.node)
                
    def tearDown(self):
        self.executor.remove_node(self.node)
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
                    response_dict["message"] = "error: aidog_rotatezaxis_absolute Unexpected content type"
                    self.node.get_logger().error(f"Unexpected content type: {response.text()}")
                    return response_dict
            else:
                if response.headers.get('Content-Type') == 'application/json':
                    response_dict = response.json()
                    if "message" not in response_dict:
                        response_dict["message"] = "error: aidog_rotatezaxis_absolute JSONrequest failed " + str(response.text)
                else:
                    response_dict["message"] = "error: aidog_rotatezaxis_absolute request failed " + str(response.text)
                self.node.get_logger().error(response_dict["message"])               
        except Exception as e:
            self.node.get_logger().error(f"Request failed: {e}")
            response_dict["message"] = "error: aidog_rotatezaxis_absolute " + str(e)
        return response_dict
    
    def rotateTo(self, method, angle, angular_velocity, num_msgs):
        msgs_rx = []
        sub = self.node.create_subscription(
                Twist, '/cmd_vel',
                lambda msg: msgs_rx.append(msg), 10)
        try:    
            request = { }
            request['turn_angle'] = angle
            request['angular_velocity'] = angular_velocity
            amount_to_turn = TestWebRotateZAxisAbsoluteService.current_angle - angle
            if (angular_velocity < 0) and (angle > TestWebRotateZAxisAbsoluteService.current_angle):  # turn through 0:
                amount_to_turn = (360 - angle) + TestWebRotateZAxisAbsoluteService.current_angle
            if (angular_velocity > 0) and (angle < TestWebRotateZAxisAbsoluteService.current_angle):  # turn through 0:
                amount_to_turn = (360 - TestWebRotateZAxisAbsoluteService.current_angle) + angle           
            duration = round(abs(amount_to_turn / angular_velocity))
            self.node.get_logger().info("current_angle: {:2f} amount_to_turn: {:2f} duration: {:2f}".format(TestWebRotateZAxisAbsoluteService.current_angle, amount_to_turn, duration))
            if method == 'post':
                response = asyncio.run(self.sendPostRequest(request))
            elif method == 'form':
                response = asyncio.run(self.sendFormRequest(request))
            else:
                response = asyncio.run(self.sendGetRequest(request))
            #self.node.get_logger().error(f"Response returned: {response}")
            self.assertIsNotNone(response)
            self.assertTrue(response['success'])
            self.assertEqual(round(response['last_angle']), angle)
            self.assertEqual(round(response['elapsed_time']), duration)
            time.sleep(1)
            self.assertGreaterEqual(len(msgs_rx), num_msgs)
            TestWebRotateZAxisAbsoluteService.current_angle = response['last_angle']
        finally:
            self.node.destroy_subscription(sub)
            
    def test_1rotateto0Get(self):
        """Test rotation to zero degrees clockwise using get"""
        self.rotateTo('get', 0, 10, 0)
            
    def test_2rotateto30Post(self):
        """Test rotate to 30 clockwise using post"""
        self.rotateTo('post', 30, 10, 1)
        
    def test_3rotateto60Form(self):
        """Test rotate to 60 clockwise using form"""
        self.rotateTo('form', 60, 10, 1)
        
    def test_4rotateto30_counter(self):
        """Test rotate to 30 counterclockwise using get"""
        self.rotateTo('get', 30, -10, 1)
    
    def test_5rotateto330_counter(self):
        """Test rotate to 330 counterclockwise using form"""
        self.rotateTo('get', 330, -10, 1)
        
    def test_6rotateto300_counter(self):
        """Test rotate to 300 counterclockwise using post"""
        self.rotateTo('post', 300, -10, 1)
        
    def test_7rotateto0(self):
        """Test rotate to 0 clockwise using get"""
        self.rotateTo('get', 0, 10, 0)