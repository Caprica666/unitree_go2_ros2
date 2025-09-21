#!/usr/bin/env python3

from aidog_interfaces.srv import RotateZAxisAbsolute
from aidog_control.aidog_rotatezaxis import RotateZAxis
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
    
class RotateZAxisAbsoluteService(RotateZAxis):
    def __init__(self):
        super().__init__('aidog_rotatezaxis_absolute_service', absolute=True)

        self.srv = self.create_service(RotateZAxisAbsolute,
                                       'aidog_rotatezaxis_absolute',
                                       self.rotatezaxis_callback,
                                       callback_group = self.callback_group)
        
    def rotatezaxis_callback(self, request, response):
        result = self.handle_rotatezaxis(request.turn_angle, request.angular_velocity, 0, 0)
        response.last_angle = result['last_angle'] 
        response.elapsed_time = result['elapsed_time']
        response.success = result['success']
        response.message = result['message']
        return response

def main(args=None):
    rclpy.init(args=args)
    aidog_rotatezaxis_absolute_service = RotateZAxisAbsoluteService()
    executor = MultiThreadedExecutor()
    executor.add_node(aidog_rotatezaxis_absolute_service)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        #aidog_rotatezaxis_absolute_service.rotatezaxis_relative(0.0)  # Ensure to stop the robot
        executor.remove_node(aidog_rotatezaxis_absolute_service)
        aidog_rotatezaxis_absolute_service.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()