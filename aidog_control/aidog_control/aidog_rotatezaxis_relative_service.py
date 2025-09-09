#!/usr/bin/env python3


from aidog_interfaces.srv import RotateZAxisRelative
from aidog_control.aidog_rotatezaxis import RotateZAxis
import rclpy
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
    
class RotateZAxisRelativeService(RotateZAxis):
    def __init__(self):
        super().__init__('aidog_rotatezaxis_relative_service')

        self.srv = self.create_service(RotateZAxisRelative,
                                       'aidog_rotatezaxis_relative',
                                       self.rotatezaxis_callback,
                                       callback_group = self.callback_group)
        self.get_logger().info('aidog_rotatezaxis_relative service is ready.')
        
    def rotatezaxis_callback(self, request, response):
        result = self.handle_rotatezaxis(request.turn_angle, request.angular_velocity, request.current_angle, request.end_angle)
        if 'last_angle' in result:
            response.last_angle = result['last_angle']
        if 'at_end' in result:
            response.atend = result['atend']
        if 'elapsed_time' in result:
            response.elapsed_time = result['elapsed_time']
        response.success = result['success']
        response.message = result['message']
        return response

def main(args=None):
    rclpy.init(args=args)
    aidog_rotatezaxis_relative_service = RotateZAxisRelativeService()
    executor = MultiThreadedExecutor()
    executor.add_node(aidog_rotatezaxis_relative_service)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        #aidog_rotatezaxis_relative_service.stop()  # Ensure to stop the robot
        executor.remove_node(aidog_rotatezaxis_relative_service)
        aidog_rotatezaxis_relative_service.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()