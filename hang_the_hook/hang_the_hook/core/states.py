import os
from traceback import print_exc

from yasmin import(
    State,
    Blackboard,
    YASMIN_LOG_INFO,
    YASMIN_LOG_ERROR,
    YASMIN_LOG_DEBUG
)
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from hang_the_hook.utils.blackboard_utils import blackboard_check

from hang_the_hook.core.constants import *

from nectar.control import(
    DroneFactory,
    MavrosConfig,
    MavlinkConfig,
    MavrosDrone,
    MavlinkDrone,
    MoveReference,
    RTLMethod,
    PIDController,
    SITL_GAZEBO_CONFIG,
)

from nectar.vision import(
    ImageHandler,
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
                else MavlinkConfig(connection_string="/dev/ttyAMA1")
            )
            drone = DroneFactory.create("mavlink", config, node._executor)

            # ---- Line Detector ----
            linedetector = LineDetector(
                color="blue_line",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.HSV,
            )

            hosedetector = LineDetector(
                color="red_hose_dawn",
                estimation_method=RotatedRect(),
                color_space=ColorSpace.LAB
            )

            # ---- Camera ----
            if SIM_MODE:
                cam_config = ROSConfig(
                    topic=IMAGE_SOURCE,
                    compressed=IMAGE_COMPRESSED,
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
                f"Camera {type(camera.camera)} ready. Frame shape: {frame.shape}."
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

            # ---- Quickly verifies altitude source ----
            while True:
                altitude = drone.get_altitude() #type: ignore # Yes, it exists, trust me
                if altitude is None:
                    altitude_reference_loss_counter += 1
                    YASMIN_LOG_DEBUG(f'Lost height reference, altitude_loss_counter: {altitude_reference_loss_counter}')
                else:
                    altitude_reference_loss_counter = 0
                    break
                drone.delay(0.1) #type: ignore # Yes, it exists, trust me

                if altitude_reference_loss_counter >= MAX_ALTITUDE_REFERENCE_LOSS:
                    YASMIN_LOG_DEBUG(f'Could not resolve altitude reference fault after {altitude_reference_loss_counter} iterations during state DESCEND, returning to launch')
                    return ABORT

            log_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "utils", "errors"))
            os.makedirs(log_dir, exist_ok=True)
            log_file = os.path.join(log_dir, "error_log.csv")
            with open(log_file, "w") as f:
                f.write("cx_error,angle_error,time\n")

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

        self.drone: MavrosDrone | MavlinkDrone

    def execute(self, blackboard: Blackboard):
        input('Press enter to Takeoff')
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                'drone',
                )
            ): return ABORT

        self.drone = blackboard["drone"]

        try:
            YASMIN_LOG_INFO(f"Taking off to {TAKEOFF_HEIGHT}m...")
            ok = self.drone.takeoff(altitude=TAKEOFF_HEIGHT, max_retries=5)
            ok = self.drone.move_to(x=0.0,y=0.0,z=(2.0 - self.drone.get_altitude()))

            YASMIN_LOG_INFO("Takeoff complete.")
            return SUCCEED if ok else ABORT

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
            ): return ABORT

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
                    ): return ABORT

        self.drone = blackboard["drone"]

        try:
            YASMIN_LOG_INFO("Landing...")
            self.drone.land()
            self.drone.delay(3)
            YASMIN_LOG_INFO("Landing complete.")

            return SUCCEED

        except Exception as e:
            YASMIN_LOG_ERROR(f"Landing failed: {e}")
            print_exc()
            return ABORT
        finally:
            if "camera" in blackboard:
                blackboard["camera"].close()
