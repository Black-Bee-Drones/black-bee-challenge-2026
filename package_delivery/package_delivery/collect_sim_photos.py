import os
import cv2
import time
import logging

import nectar
from nectar.vision import ImageHandler, ROSConfig

import rclpy
from rclpy.node import Node

from package_delivery.constants import Config

log = logging.getLogger("photo_collector")


class PhotoCollector(Node):
    def __init__(self, config=Config, output_folder="sim_collected_photos", capture_interval=1.0):
        super().__init__('photo_collector')
        nectar.add_node(self)

        self.config = config
        self.output_folder = output_folder
        self.frame_number = 0
        self.capture_interval = capture_interval
        self._last_capture = 0.0

        os.makedirs(self.output_folder, exist_ok=True)

        # --- Initializing Camera ---
        cam_config = ROSConfig(
            topic=self.config.sim_image_source,
            compressed=self.config.sim_image_compressed,
        )
        self.camera = ImageHandler(
            image_source=self.config.sim_image_source,
            config=cam_config,
            image_processing_callback=self.photo_callback,
            poll_interval=0.01,
        )

    def photo_callback(self, image):
        if image is None:
            return

        now = time.time()

        if now - self._last_capture < self.capture_interval:
            return
        self._last_capture = now

        filename = f"{self.frame_number:03d}_frame.jpg"
        path = os.path.join(self.output_folder, filename)

        if cv2.imwrite(path, image):
            self.frame_number += 1
            log.info(f"[{self.frame_number}] Saved {filename}")
        else:
            log.warning(f"Failed to write frame to {path}")

    def start(self):
        self.camera.open()
        self.camera.run()

    def stop(self):
        self.camera.cleanup()
        path = os.path.abspath(self.output_folder)
        log.info(f"\033[95m Collection finished: {self.frame_number} photos saved to {path}\033[0m")


def main():
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    nectar.init()
    collector = PhotoCollector()

    try:
        collector.start()
        nectar.spin()
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        collector.stop()
        nectar.shutdown()


if __name__ == "__main__":
    main()