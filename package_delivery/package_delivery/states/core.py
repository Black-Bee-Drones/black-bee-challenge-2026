import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

import nectar
from nectar.control import (
    DroneFactory,
    MavrosDrone,
    MavlinkDrone,
    MavrosConfig,
    MavlinkConfig,
    PoseSource,
    RTLMethod,
    SITL_GAZEBO_CONFIG,
    PIDController,
)
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision.camera import ROSConfig

from package_delivery.constants import Config


class Initialize(State):
    def __init__(self, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Setting up initializing options...")
            if (self.config.sim_mode):
                drone_config = SITL_GAZEBO_CONFIG
                cam_config = ROSConfig(
                    topic=self.config.sim_image_source, 
                    compressed=self.config.sim_image_compressed,
                    )
            else:
                drone_config = MavrosConfig(pose_source=PoseSource.GPS)
                cam_config = OpenCVConfig(
                    width=self.config.image_width, 
                    height=self.config.image_height,
                    )

            yasmin.YASMIN_LOG_INFO("Initializing drone...")
            drone = DroneFactory.create("mavros", drone_config)
            blackboard["drone"] = drone

            yasmin.YASMIN_LOG_INFO("Initializing camera...")
            camera = ImageHandler(
                image_source=self.config.sim_image_source, 
                config=cam_config,
            )

            camera.open()
            frame = camera.take_photo()
            camera.cleanup()

            if frame is None:
                yasmin.YASMIN_LOG_ERROR("Failed to get frame from camera. Aborting...")
                return ABORT

            yasmin.YASMIN_LOG_INFO(
                f"Camera ready. Frame shape: {frame.shape}"
            )
            blackboard["camera"] = camera

            yasmin.YASMIN_LOG_INFO("Initializing PID Controller...")
            pid_cx = PIDController(
                kp=self.config.x_kp,
                ki=self.config.x_ki,
                kd=self.config.x_kd,
                output_limits=self.config.xy_output_lim,
                integral_limits=self.config.xy_integral_lim,
            )
            pid_cy = PIDController(
                kp=self.config.y_kp,
                ki=self.config.y_ki,
                kd=self.config.y_kd,
                output_limits=self.config.xy_output_lim,
                integral_limits=self.config.xy_integral_lim,
            )
            blackboard["pid_cx"] = pid_cx
            blackboard["pid_cy"] = pid_cy

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT


class Takeoff(State):
    def __init__(self, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config

    def execute(self, blackboard: Blackboard):
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')
        
        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
        
        else:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT

        yasmin.YASMIN_LOG_INFO(
            f'Taking off to altitude: {self.config.takeoff_altitude} m...')

        try:
            drone.takeoff(self.config.takeoff_altitude)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Taking off failed: {e}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED


class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
        
        else:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT

        yasmin.YASMIN_LOG_INFO('Landing...')

        try:
            drone.land()

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Landing failed: {e}.')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED


class Rtl(State):
    def __init__(self, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        
        self.config = config
    
    def execute(self, blackboard: Blackboard):
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
        
        else:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT

        yasmin.YASMIN_LOG_INFO("RTL...")
        try:
            self.drone.rtl(altitude=self.config.rtl_altitude, RTLMethod=RTLMethod.NAVIGATE, land=True)
            yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT