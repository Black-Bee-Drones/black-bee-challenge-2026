# Code to open a window to see the drone's camera view
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
import rclpy
from rclpy.node import Node

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage

class CameraViewer(Node):
    def __init__(self):
        super().__init__('camera_viewer')

        #Declare if the Image will be compressed or not
        self.declare_parameter("use_compression", True)
        self.use_compression = self.get_parameter("use_compression").value

        self.bridge = CvBridge()

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        if self.use_compression:
            self.subscription = self.create_subscription(
                CompressedImage,
                'image_raw/compressed', #Name of the topic
                self.compressed_callback,
                qos_profile
            )
            self.get_logger().info("Subscribed to compressed image")
        else:
            self.subscription = self.create_subscription(
                Image,
                'image_raw', #Again the name of the topic
                self.image_callback,
                qos_profile,
            )

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        if frame is None:
            return
        
        cv2.imshow('Camera', frame)
        cv2.waitKey(1)
    
    def compressed_callback(self, msg):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        if frame is None:
            return
        
        cv2.imshow('Camera', frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = CameraViewer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if (__name__ == '__main__'):
    main()