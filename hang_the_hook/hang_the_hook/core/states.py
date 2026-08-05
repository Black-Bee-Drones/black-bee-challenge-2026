from dataclasses import dataclass, asdict
from traceback import print_exc

from yasmin import(
    State,
    Blackboard,
    YASMIN_LOG_INFO,
    YASMIN_LOG_ERROR,
    YASMIN_LOG_WARN
)
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from hang_the_hook.core.constants import (
    RTL_ALTITUDE,
    TAKEOFF_HEIGHT,
    SIM_MODE,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    IMAGE_SOURCE,
    SIM_IMAGE_COMPRESSED,
    BASE_PID_DICT,
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

            # ---- Line Detector ----
            linedetector = LineDetector(
                color="blue",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.HSV
                )

            hosedetector = LineDetector(
                color="red",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.HSV
                )

            # ---- PID ----
            pid_config = PIDConfig.from_dict(BASE_PID_DICT)
            pid_cx, pid_cy, pid_angle = (PIDController(**asdict(pid_config)) for _ in range(3))

            # ---- Blackboard ----
            blackboard["drone"]       = drone
            blackboard["camera"]      = camera
            blackboard["line_detect"] = linedetector
            blackboard["hose_detect"] = hosedetector
            blackboard["pid_cx"]      = pid_cx
            blackboard["pid_cy"]      = pid_cy
            blackboard["pid_angle"]   = pid_angle

            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"Init error {e}")
            print_exc()
            return ABORT

class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED,ABORT])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            YASMIN_LOG_ERROR("Drone not available.")
            return ABORT

        drone: MavrosDrone | MavlinkDrone= blackboard["drone"]

        try:
            YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_HEIGHT}m...")
            drone.set_home()
            drone.arm()
            drone.takeoff(TAKEOFF_HEIGHT)
            drone.delay(3)

            reached = drone.move_to(
                z=TAKEOFF_HEIGHT,
                reference=MoveReference.TAKEOFF,
                timeout=30.0,
                precision=0.3,
            )

            if not reached:
                YASMIN_LOG_WARN("Takeoff move_to timed out, continuing.")

            drone.delay(1)
            YASMIN_LOG_INFO("Takeoff complete.")
            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"Takeoff failed: {e}")
            return ABORT

class ReturnToLaunch(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            YASMIN_LOG_ERROR("Drone not available.")
            return ABORT

        drone: MavrosDrone | MavlinkDrone = blackboard["drone"]

        try:
            YASMIN_LOG_INFO(f"Returning to launch at {RTL_ALTITUDE}m...")
            drone.rtl(
                altitude=RTL_ALTITUDE,
                method=RTLMethod.NAVIGATE,
                land=False,
            )
            drone.delay(2)
            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT

class End(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            YASMIN_LOG_ERROR("Drone not available.")
            return ABORT

        drone: MavrosDrone | MavlinkDrone = blackboard["drone"]

        try:
            YASMIN_LOG_INFO("Landing...")
            drone.land()
            drone.delay(3)
            YASMIN_LOG_INFO("Landing complete.")

            if "camera" in blackboard:
                blackboard["camera"].close()

            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"Landing failed: {e}")
            return ABORT
