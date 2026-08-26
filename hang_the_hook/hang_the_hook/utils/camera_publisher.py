import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from hang_the_hook.core.constants import (
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    IMAGE_SOURCE
)

from nectar.vision import (
    ImageHandler,
    OpenCVConfig
)

class CameraPublisher(Node):

    def __init__(self) -> None:
        super().__init__('camera_publisher')
        self.publisher = self.create_publisher(Image, 'camera', 10)

        self.bridge = CvBridge()

        cam_config = OpenCVConfig(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)

        self.camera = ImageHandler(
            image_source=IMAGE_SOURCE,
            config=cam_config,
        )

        timer_period = 1.0 / 30.0
        self.timer = self.create_timer(timer_period, self.publish_image)

    def publish_image(self) -> None:
        frame = self.camera.take_photo()

        if frame is not None:
            img_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_frame'

            self.publisher.publish(img_msg)
