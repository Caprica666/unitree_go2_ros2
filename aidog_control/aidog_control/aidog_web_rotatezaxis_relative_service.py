#!/usr/bin/env python3


from ros2web_interfaces.srv import HTTP
from aidog_control.aidog_web_rotatezaxis import RotateZAxisWebService
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.executors import MultiThreadedExecutor
   
class RotateZAxisRelativeService(RotateZAxisWebService):
    def __init__(self):
        super().__init__('aidog_rotatezaxis_relative_service', absolute=False)
        self.get_srv = self.create_service(HTTP,
                                       'http/get/aidog_rotatezaxis_relative',
                                       self.process_get_request,
                                       callback_group = self.callback_group)
        self.post_srv = self.create_service(HTTP,
                                'http/post/aidog_rotatezaxis_relative',
                                self.process_post_request,
                                callback_group = self.callback_group)

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
        #aidog_rotatezaxis_relative_service.rotatezaxis_relative(0.0)  # Ensure to stop the robot
        executor.remove_node(aidog_rotatezaxis_relative_service)
        aidog_rotatezaxis_relative_service.destroy_node()
        executor.shutdown()
        rclpy.shutdown()

if __name__ == '__main__':
    main()