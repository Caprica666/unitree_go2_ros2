
import unittest
import pytest
import rclpy
from aidog_interfaces.srv import RotateZAxisRelative
from rosgraph_msgs.msg import Clock
from geometry_msgs.msg import Twist
import launch
import launch_ros.actions
import launch_testing.actions
import launch_testing.markers
from launch_testing.io_handler import ActiveIoHandler
from rclpy.executors import MultiThreadedExecutor

@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    launch_service = launch_ros.actions.Node(
            executable='aidog_rotatezaxis_relative_service',
            package='aidog_control',
            output='screen'
        )
    service_under_test = launch.actions.ExecuteProcess(
        cmd=[
            "ros2",
            "run",
            "aidog_control",
            'aidog_rotatezaxis_relative_service',
            ],
        output='screen'
    )
    wait_for_service = launch.actions.TimerAction(
        period=2.0,
        actions=[launch_testing.actions.ReadyToTest()]
    )
    launch_desc = launch.LaunchDescription(
            [
                launch_service,
                wait_for_service,
            ]
        )
    return launch_desc

class ClockServer():
    def __init__(self, node):
        self.node = node
        self.pub = self.node.create_publisher(Clock, '/clock', 1)
        self.timer = self.node.create_timer(0.1, self.timer_callback,
                                            callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.node.get_logger().info('Starting clock server')
        
    def get_time(self):
        t = self.node.get_clock().now()
        seconds, nanos = t.seconds_nanoseconds()
        seconds += float(nanos) * 1e-9  # Convert nanoseconds to seconds
        return seconds
      
    def timer_callback(self):
        clockmsg = Clock()
        current_time = self.get_time()
        clockmsg.clock = rclpy.time.Time(seconds=current_time).to_msg()
        #self.node.get_logger().info(f'timer_callback {current_time}')
        self.pub.publish(clockmsg)
        
    def stop(self):
        self.node.destroy_publisher(self.pub)    
    
class TestRotateZAxisRelativeService(unittest.TestCase):
    @classmethod
    def setUpClass(self):
        # Initialize the ROS context for the test node
        rclpy.init()
        self.executor = MultiThreadedExecutor()
        self.callback_group = rclpy.callback_groups.ReentrantCallbackGroup()

        
    @classmethod
    def tearDownClass(self):
        # Shutdown the ROS context
        self.executor.shutdown()
        rclpy.shutdown()
                
    def setUp(self):
        self.node = rclpy.create_node('test_aidog_rotatezaxis_relative')
        self.executor.add_node(self.node)
        self.clock = ClockServer(self.node)
                
    def tearDown(self):
        self.clock.stop()
        self.executor.remove_node(self.node)
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
        self.executor.spin_until_future_complete(future)
        if future.result() is not None:
            self.node.get_logger().info(future.result().message)
        else:
            self.node.get_logger().error('Service call failed.')
        return future.result()
            
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
            request = RotateZAxisRelative.Request()
            deg2rad = (3.14159 / 180)
            request.turn_angle = 30.0 * deg2rad
            request.current_angle = 0.0
            request.angular_velocity = 10.0 * deg2rad
            request.end_angle = 180.0 * deg2rad
            duration = abs(request.turn_angle / request.angular_velocity)

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertEqual(response.elapsed_time, duration)
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
            request = RotateZAxisRelative.Request()
            deg2rad = (3.14159 / 180)
            request.turn_angle = -30.0 * deg2rad
            request.current_angle = 0.0
            request.angular_velocity = 10.0 * deg2rad
            request.end_angle = -180.0 * deg2rad
            duration = abs(request.turn_angle / request.angular_velocity)

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertTrue(response.success)
            self.assertEqual(response.elapsed_time, duration)
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
            request = RotateZAxisRelative.Request()
            deg2rad = (3.14159 / 180)
            request.turn_angle = -30.0 * deg2rad
            request.current_angle = 0.0
            request.angular_velocity = 10.0 * deg2rad
            request.end_angle = 180.0 * deg2rad
            duration = abs(request.turn_angle / request.angular_velocity)

            response = self.sendRequest(client, request)
            self.assertIsNotNone(response)
            self.assertFalse(response.success)
            self.assertIn('error', response.message)
        finally:
            self.node.destroy_client(client)
            self.node.destroy_subscription(sub)

