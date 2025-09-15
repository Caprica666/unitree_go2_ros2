import rclpy
from rosgraph_msgs.msg import Clock

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
        self.timer.cancel()
        self.node.destroy_timer(self.timer)
        self.node.destroy_publisher(self.pub)
        self.node.get_logger().info('Stopping clock server')    
    