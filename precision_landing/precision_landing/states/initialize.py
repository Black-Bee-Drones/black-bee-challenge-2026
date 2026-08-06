import os
import cv2

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

import nectar
from nectar.control import (
    DroneFactory,
    MavrosConfig,
    PoseSource,
    SITL_GAZEBO_CONFIG,
)
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision.camera import ROSConfig
from nectar.ai import Detector
#TODO: create the detector and save it in the blackboard

from precision_landing.constants import (
    SIM_MODE,
    CAMERA_SOURCE,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    FRAMES_FOLDER,
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            node = YasminNode.get_instance()
            yasmin.YASMIN_LOG_INFO("Initializing Drone...")

            config = (
                SITL_GAZEBO_CONFIG if SIM_MODE
                else MavrosConfig(pose_source=PoseSource.GPS, expect_lidar=True)
            )
            drone = DroneFactory.create("mavros", config, node._executor)
            blackboard["drone"] = drone

            if SIM_MODE:
                cam_config = ROSConfig(topic=CAMERA_SOURCE, compressed=False)
            else:
                cam_config = OpenCVConfig(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)

            camera = ImageHandler(
                image_source=CAMERA_SOURCE,
                config=cam_config,
                image_processing_callback=self.camera_callback,
                #NOTE: There are other ways of doing the continuously image processing,
                #this is one of them, but we can discuss this later
            )
            camera.open()
            camera.run()
            frame = camera.take_photo()
            if frame is None:
                yasmin.YASMIN_LOG_ERROR("Failed to get frame from camera.")
                return ABORT
            yasmin.YASMIN_LOG_INFO(
                f"Camera ready. Frame Shape: {frame.shape}"
            )
            blackboard["camera"] = camera

            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT

    def camera_callback(self, image):
        photos_folder = FRAMES_FOLDER

        #os.makedirs(photos_folder, exist_ok=True)

        #raw_path = os.path.join(photos_folder, 'frame.png')
        #cv2.imwrite(raw_path, image)

        return image