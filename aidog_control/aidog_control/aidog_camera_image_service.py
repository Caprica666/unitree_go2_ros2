import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from aidog_interfaces.srv import GetCameraImage

class CameraImageService(Node):
    def __init__(self):
        super().__init__('aidog_camera_image_service')
        self.current_image = None
        self.image_sub = self.create_subscription(
            Image,
            'rgb_image',
            self.image_callback,
            10
        )
        self.srv = self.create_service(GetCameraImage, 'aidog_get_camera_image', self.handle_get_image)
        self.get_logger().info('CameraImageService node started, listening to rgb_image and providing get_camera_image service.')

    def image_callback(self, msg):
        self.current_image = msg

    def handle_get_image(self, request, response):
        if self.current_image is not None:
            self.get_logger().info('Returning latest image.')
            response.image = self.current_image
        else:
            self.get_logger().warn('No image received yet. Returning empty image.')
            response.image = Image()
        return response

def main(args=None):
    rclpy.init(args=args)
    node = CameraImageService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
