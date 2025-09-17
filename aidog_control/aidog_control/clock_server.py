import rclpy
from rclpy.node import Node
from rosgraph_msgs.msg import Clock

class ClockServer(Node):
    def __init__(self):
        super().__init__('clock_server')
        self.pub = self.create_publisher(Clock, '/clock', 1)
        self.timer = self.create_timer(0.1, self.timer_callback,
                                            callback_group=rclpy.callback_groups.ReentrantCallbackGroup())
        self.get_logger().info('Starting clock server')
        
    def get_time(self):
        t = self.get_clock().now()
        seconds, nanos = t.seconds_nanoseconds()
        seconds += float(nanos) * 1e-9  # Convert nanoseconds to seconds
        return seconds
      
    def timer_callback(self):
        clockmsg = Clock()
        current_time = self.get_time()
        clockmsg.clock = rclpy.time.Time(seconds=current_time).to_msg()
        #self.node.get_logger().info(f'timer_callback {current_time}')
        self.pub.publish(clockmsg)
        

def main():
    rclpy.init()
    node = ClockServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()   
    