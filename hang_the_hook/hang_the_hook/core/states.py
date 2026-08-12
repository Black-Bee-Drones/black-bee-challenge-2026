import os
from dataclasses import dataclass, asdict
from traceback import print_exc
from typing import Any

from yasmin import(
    State,
    Blackboard,
    YASMIN_LOG_INFO,
    YASMIN_LOG_ERROR,
    YASMIN_LOG_WARN
)
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from hang_the_hook.utils.blackboard_utils import blackboard_check

from hang_the_hook.core.constants import (
    RTL_ALTITUDE,
    TAKEOFF_HEIGHT,
    SIM_MODE,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    IMAGE_SOURCE,
    SIM_IMAGE_COMPRESSED,
    PID_X_KP, PID_X_KI, PID_X_KD, PID_X_OUTPUT_LIMITS, PID_X_INTEGRAL_LIMITS,
    PID_Y_KP, PID_Y_KI, PID_Y_KD, PID_Y_OUTPUT_LIMITS, PID_Y_INTEGRAL_LIMITS,
    PID_YAW_KP, PID_YAW_KI, PID_YAW_KD, PID_YAW_OUTPUT_LIMITS, PID_YAW_INTEGRAL_LIMITS,
)

from nectar.control import(
    DroneFactory,
    MavrosConfig,
    MavlinkConfig,
    MavrosDrone,
    MavlinkDrone,
    PoseSource,
    MoveReference,
    RTLMethod,
    PIDController,
    PIDConfig,
    SITL_GAZEBO_CONFIG,
)

from nectar.vision import(
    ImageHandler,
    CameraFactory,
    OpenCVConfig,
    LineDetector,
    RotatedRect,
    ColorSpace,
)
from nectar.vision.camera import ROSConfig
from nectar.vision.algorithms.color import ColorDetector

class Initialize(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        try:
            # ---- Yasmin ----
            node = YasminNode.get_instance()
            YASMIN_LOG_INFO("Initializing drone...")

            # ---- Nectar ----
            config = (
                SITL_GAZEBO_CONFIG if SIM_MODE
                else MavrosConfig(pose_source=PoseSource.GPS)
            )
            drone = DroneFactory.create("mavros", config, node._executor)

            # ---- Line Detector ----
            linedetector = LineDetector(
                color="blue",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.HSV,
            )

            hosedetector = LineDetector(
                color="red",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.LAB
            )

            # ---- Camera ----
            if SIM_MODE:
                cam_config = ROSConfig(
                    topic=IMAGE_SOURCE,
                    compressed=SIM_IMAGE_COMPRESSED,
                )
            else:
                cam_config = OpenCVConfig(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)

            camera = ImageHandler(
                image_source=IMAGE_SOURCE,
                config=cam_config,
            )

            camera.open()
            frame = camera.take_photo(timeout_sec=15)
            if frame is None:
                YASMIN_LOG_ERROR("Failed to get frame from camera.")
                return ABORT

            YASMIN_LOG_INFO(
                f"Camera {type(camera)} ready. Frame shape: {frame.shape}."
            )

            # ---- Line Detector (linha azul — preset HSV do Nectar) ----
            linedetector = LineDetector(
                color="blue",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.LAB,
            )

            # ---- Hose Detector (mangueira vermelha — preset LAB do Nectar) ----
            hosedetector = LineDetector(
                color="red",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.LAB,
            )

            # ---- PID ----
            pid_cx = PIDController(
                kp=PID_X_KP, ki=PID_X_KI, kd=PID_X_KD,
                output_limits=PID_X_OUTPUT_LIMITS,
                integral_limits=PID_X_INTEGRAL_LIMITS,
            )
            pid_cy = PIDController(
                kp=PID_Y_KP, ki=PID_Y_KI, kd=PID_Y_KD,
                output_limits=PID_Y_OUTPUT_LIMITS,
                integral_limits=PID_Y_INTEGRAL_LIMITS,
            )
            pid_angle = PIDController(
                kp=PID_YAW_KP, ki=PID_YAW_KI, kd=PID_YAW_KD,
                output_limits=PID_YAW_OUTPUT_LIMITS,
                integral_limits=PID_YAW_INTEGRAL_LIMITS,
            )

            # ---- Blackboard ----
            blackboard["drone"]       = drone
            blackboard["camera"]      = camera
            blackboard["line_detect"] = linedetector
            blackboard["hose_detect"] = hosedetector
            blackboard["pid_cx"]      = pid_cx
            blackboard["pid_cy"]      = pid_cy
            blackboard["pid_angle"]   = pid_angle
            blackboard['findhose_state_counter'] = 0

            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"Init error {e}")
            print_exc()
            return ABORT

class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED,ABORT])

        self.drone: MavrosDrone | MavlinkDrone

    def execute(self, blackboard: Blackboard):
        if not blackboard_check(
                    blackboard=blackboard,
                    args=(
                        'drone',
                        )
                    ):
                    return ABORT

        self.drone = blackboard["drone"]

        try:
            YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_HEIGHT}m...")
            self.drone.set_home()
            self.drone.arm()
            self.drone.takeoff(TAKEOFF_HEIGHT)
            self.drone.delay(3)

            reached = self.drone.move_to(
                z=TAKEOFF_HEIGHT,
                reference=MoveReference.TAKEOFF,
                timeout=30.0,
                precision=0.3,
            )

            reached = self.drone.move_to(
                x=0.3,
                reference=MoveReference.BODY,
                timeout=10.0,
                precision=0.3,
            )

            if not reached:
                YASMIN_LOG_WARN("Takeoff move_to timed out, continuing.")

            self.drone.delay(1)
            YASMIN_LOG_INFO("Takeoff complete.")
            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            print_exc()
            return ABORT

class ReturnToLaunch(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.drone: MavrosDrone | MavlinkDrone

    def execute(self, blackboard: Blackboard):
        if not blackboard_check(
                    blackboard=blackboard,
                    args=(
                        'drone',
                        )
                    ):
                    return ABORT

        self.drone = blackboard["drone"]

        try:
            YASMIN_LOG_INFO(f"Returning to launch at {RTL_ALTITUDE}m...")
            self.drone.rtl(
                altitude=RTL_ALTITUDE,
                method=RTLMethod.NAVIGATE,
                land=False,
            )
            self.drone.delay(2)
            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT

class End(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.drone: MavrosDrone | MavlinkDrone

    def execute(self, blackboard: Blackboard):
        if not blackboard_check(
                    blackboard=blackboard,
                    args=(
                        'drone',
                        )
                    ):
                    return ABORT

        self.drone = blackboard["drone"]

        try:
            YASMIN_LOG_INFO("Landing...")
            self.drone.land()
            self.drone.delay(3)
            YASMIN_LOG_INFO("Landing complete.")

            if "camera" in blackboard:
                blackboard["camera"].close()

            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT
