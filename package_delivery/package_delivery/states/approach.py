import time
import math

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

import nectar
from nectar.control import (
    MavrosDrone,
    PIDController,
)
from nectar.vision import (
    ImageHandler,
)

from package_delivery.constants import Config


class Approach(State):
    def __init__(self, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard: 
            yasmin.YASMIN_LOG_ERROR("Drone not available.")
            return ABORT
        drone: MavrosDrone = blackboard["drone"]

        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available.")
            return ABORT
        camera: ImageHandler = blackboard["camera"]

        if "pid_cx" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("X PID Controller not available.")
            return ABORT
        if "pid_cy" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Y PID Controller not available.")
            return ABORT
        pid_cx: PIDController = blackboard["pid_cx"]
        pid_cy: PIDController = blackboard["pid_cy"]

        pid_cx.set_setpoint = self.config.image_width//2
        pid_cy.set_setpoint = self.config.image_height//2

        try:
            start_time = time.time()

            while True: 
                frame = camera.take_photo

                # TODO: implementar o detector

                if (time.time() - start_time) > self.config.approach_timeout:
                    yasmin.YASMIN_LOG_ERROR("Approaching box timed out.")
                    return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Approaching box failed: {e}")
            return ABORT


    def ppm(self, delta_pixel: int, altitude: float, fov_degrees: float, frame_px: int) -> float:

        angle_rad = math.radians(fov_degrees)/2

        ratio = (math.tan(angle_rad) * altitude) / (frame_px//2)

        return delta_pixel*ratio
