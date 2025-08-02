import rclpy
from rclpy.node import Node
from aidog_interfaces.srv import GetCameraImage
from sensor_msgs.msg import Image
from PIL import Image as PILImage
import numpy as np
import sys

class CameraImageClient(Node):
    def __init__(self, width=None, height=None):
        super().__init__('camera_image_client')
        self.cli = self.create_client(GetCameraImage, 'aidog_camera_image')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.req = GetCameraImage.Request()
        self.desired_width = width
        self.desired_height = height

    def send_request(self):
        future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            img_msg = future.result().image
            if img_msg.data:
                self.display_image(img_msg)
            else:
                self.get_logger().warn('No image data received from service.')
        else:
            self.get_logger().error('Service call failed.')

    def display_image(self, img_msg: Image):
        # Only supports 'rgb8' and 'mono8' encodings
        if img_msg.encoding == 'rgb8':
            dtype = np.uint8
            channels = 3
        elif img_msg.encoding == 'mono8':
            dtype = np.uint8
            channels = 1
        else:
            self.get_logger().error(f'Unsupported encoding: {img_msg.encoding}')
            return
        img_np = np.frombuffer(img_msg.data, dtype=dtype)
        img_np = img_np.reshape((img_msg.height, img_msg.width, channels))
        pil_img = PILImage.fromarray(img_np)
        if self.desired_width and self.desired_height:
            pil_img = pil_img.resize((self.desired_width, self.desired_height))
        pil_img.show()
        # Wait for the image window to close before shutting down
        try:
            while True:
                if not any([w.is_alive() for w in PILImage._showxv_windows.values()]):
                    break
        except Exception:
            pass
        self.get_logger().info('Image window closed. Shutting down client.')
        self.destroy_node()
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    width = None
    height = None
    if len(sys.argv) > 2:
        width = int(sys.argv[1])
        height = int(sys.argv[2])
    client = CameraImageClient(width, height)
    client.send_request()
    # No need to call destroy_node() or rclpy.shutdown() here, handled in display_image

if __name__ == '__main__':
    main()
