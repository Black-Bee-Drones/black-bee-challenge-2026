# Code to take photos with the drone's camera
#
# Utility class (not a YASMIN State) meant to be instantiated and
# controlled from inside another state (e.g. Find), saving frames from
# the drone's camera at fixed time intervals while the drone flies over
# the target area. The saved images are later labeled and used to train
# a YOLO model, so we avoid saving every single frame (too many near-
# duplicates) and resize each frame to the resolution defined in
# constants.py.

import os
import time
from datetime import datetime

import cv2
import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, CompressedImage
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

import yasmin
from yasmin_ros.yasmin_node import YasminNode

from constants import (
    CAMERA_SOURCE,
    PHOTOS_FOLDER,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
)


class PhotoTaker:
    """
    Subscribes to the drone's camera topic and saves one resized frame
    to disk every `interval` seconds while capture is active.

    Typical usage inside a state's execute():

        photo_taker = PhotoTaker(interval=2.0)
        photo_taker.start()
        ... search / flight logic here ...
        photo_taker.stop()
    """

    def __init__(self, interval: float = 2.0, use_compression: bool = False):
        # Reuses the shared YASMIN ROS2 node (singleton) instead of
        # creating a new rclpy Node, so it doesn't conflict with the
        # rest of the state machine.
        self.node = YasminNode()

        self.bridge = CvBridge()
        self.interval = interval
        self.use_compression = use_compression

        self._active = False
        self._last_saved = 0.0
        self._count = 0

        if not PHOTOS_FOLDER:
            yasmin.YASMIN_LOG_ERROR(
                "PHOTOS_FOLDER is empty in constants.py - set a valid path "
                "before using PhotoTaker."
            )
        os.makedirs(PHOTOS_FOLDER, exist_ok=True)

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        if self.use_compression:
            msg_type = CompressedImage
            topic = f"{CAMERA_SOURCE}/compressed"
            callback = self._compressed_callback
        else:
            msg_type = Image
            topic = CAMERA_SOURCE
            callback = self._image_callback

        self.subscription = self.node.create_subscription(
            msg_type,
            topic,
            callback,
            qos_profile,
        )

        yasmin.YASMIN_LOG_INFO(f"PhotoTaker subscribed to '{topic}'")

    def start(self):
        """Starts saving frames at the configured interval."""
        self._active = True
        self._last_saved = 0.0
        yasmin.YASMIN_LOG_INFO("PhotoTaker: capture started")

    def stop(self):
        """Stops saving frames (subscription stays alive, just idle)."""
        self._active = False
        yasmin.YASMIN_LOG_INFO(
            f"PhotoTaker: capture stopped ({self._count} photos saved)"
        )

    @property
    def count(self) -> int:
        """Number of photos saved so far."""
        return self._count

    def _image_callback(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self._maybe_save(frame)

    def _compressed_callback(self, msg: CompressedImage):
        np_arr = np.frombuffer(msg.data, np.uint8)
        frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        self._maybe_save(frame)

    def _maybe_save(self, frame):
        if not self._active or frame is None:
            return

        now = time.time()
        if now - self._last_saved < self.interval:
            return

        try:
            frame = cv2.resize(frame, (IMAGE_WIDTH, IMAGE_HEIGHT))

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
            filename = os.path.join(
                PHOTOS_FOLDER, f"foto_{self._count:04d}_{timestamp}.jpg"
            )
            cv2.imwrite(filename, frame)

            self._count += 1
            self._last_saved = now
            yasmin.YASMIN_LOG_INFO(f"PhotoTaker: saved {filename}")

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"PhotoTaker: failed to save frame: {e}")
