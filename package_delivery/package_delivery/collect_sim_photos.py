import rclpy
import cv2 as cv
import os
import time

import nectar
from nectar.vision import ImageHandler, ROSConfig
from package_delivery.constants import Config

def main():
    config = Config
    rclpy.init()
    frame_number = 0

    # --- Initializing Camera ---
    cam_config = ROSConfig(
        topic=config.sim_image_source,
        compressed=config.sim_image_compressed,
    )
    camera = ImageHandler(
        image_source=config.sim_image_source,
        config=cam_config,
        image_processing_callback=photo_callback,
    )

    camera.open()
    camera.run()

    try:
        nectar.spin()
    except (KeyboardInterrupt, SystemExit): 
        pass
    finally:
        camera.cleanup()
        rclpy.shutdown()

def photo_callback(image):
    photos_folder = "sim_collected_photos"
    os.makedirs(photos_folder, exist_ok=True)

    filename = f"{time.time()*1000}_frame.png"
    path = os.path.join(photos_folder, filename)
    cv.imwrite(path, image)

    return


if (__name__ == "__main__"):
    main()