from ros2web_interfaces.srv import HTTP
from ros2web_interfaces.msg import ContentType, BodyPart

import io
from PIL import Image
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image as SensorImage
from aidog_interfaces.srv import GetCameraImage

class WebCameraImageService(Node):
    def __init__(self):
        super().__init__('aidog_get_camera_image')
        self.current_image = None
        self.image_sub = self.create_subscription(
            SensorImage,
            'rgb_image',
            self.image_callback,
            10
        )
        self.get_srv = self.create_service(HTTP, 'http/get/aidog_get_camera_image', self.get_callback)
        self.get_logger().info('CameraImageService node started, listening to rgb_image and providing get_camera_image service.')

    # Implement the callback for the HTTP GET request
    # This will return the latest image as a PNG file
    # or an error message if no image is available
    # The response will be in the format expected by the HTTP service
    # The image will be returned as a binary stream in the response body
    # The content type will be set to 'image/png'
    # If no image is available, a 404 status code will be returned
    # with a plain text message indicating that no image is available
    # If an image is available, it will be returned with a 200 status code
    # and the content type will be set to 'image/png'
    # The image will be converted to a PNG format before being sent
    # The response will include the image data in the body
    def get_callback(self, request: HTTP.Request, response: HTTP.Response):
        self.get_logger().info('Requesting image from camera...')
        if self.current_image is not None:
            img_io = io.BytesIO(self.current_image.data)
            pilimg = Image.frombytes('RGB', (self.current_image.width, self.current_image.height), img_io.getvalue())
            img_io = io.BytesIO()
            pilimg.save(img_io, format='PNG')
            img_io.seek(0)
            response.body = img_io.getvalue()
            response.content_type = ContentType.IMAGE_PNG
            response.status = 200
            self.get_logger().info('Returning latest image as PNG.')
        else:
            self.get_logger().warn('No image received yet. Returning empty response.')
            response.status = 404
            response.body = b''
            response.reason = "No image available"
            response.content_type = ContentType.TEXT_PLAIN
        return response
    # The image callback will be called whenever a new image is received
    # It will update the current_image attribute with the latest image
    # This will ensure that the latest image is always available for the HTTP GET request
    # The image will be stored in the current_image attribute as a PIL Image object
    # This will allow the image to be easily converted to PNG format when needed
    def image_callback(self, msg):
        self.current_image = msg

def main(args=None):
    rclpy.init(args=args)
    node = WebCameraImageService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
