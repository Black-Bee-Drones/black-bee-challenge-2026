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

from package_delivery.constants import (
    SIM_MODE,
    SIM_IMAGE_SOURCE,
    SIM_IMAGE_COMPRESSED,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
)


class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Setting up initializing options...")
            if (SIM_MODE):
                drone_config = SITL_GAZEBO_CONFIG
                cam_config = ROSConfig(
                    topic=SIM_IMAGE_SOURCE, 
                    compressed=SIM_IMAGE_COMPRESSED,
                )
            else:
                drone_config = MavrosConfig(pose_source=PoseSource.GPS)
                cam_config = OpenCVConfig(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)

            yasmin.YASMIN_LOG_INFO("Initializing drone...")
            drone = DroneFactory.create("mavros", drone_config)
            blackboard["drone"] = drone

            yasmin.YASMIN_LOG_INFO("Initializing camera...")
            camera = ImageHandler(
                image_source=SIM_IMAGE_SOURCE, 
                config=cam_config,
                image_processing_callback=self.photo_callback,
            )

            camera.open()
            frame = camera.take_photo()

            if frame is None:
                yasmin.YASMIN_LOG_ERROR("Failed to get frame from camera. Aborting...")
                return ABORT

            yasmin.YASMIN_LOG_INFO(
                f"Camera ready. Frame shape: {frame.shape}"
            )
            blackboard["camera"] = camera
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT

    def photo_callback(self, image):
        # Implement later
        return image
