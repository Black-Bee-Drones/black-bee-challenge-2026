import yasmin
import numpy as np
import cv2
import os
import pathlib

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from datetime import datetime

import nectar
from nectar.control import (
    DroneFactory,
    MavrosDrone,
    MavlinkDrone,
    MavrosConfig,
    MavlinkConfig,
    PoseSource,
    PIDController,
    RTLMethod,
    SITL_GAZEBO_CONFIG,
    PIDController,
)
from nectar.vision import ImageHandler, ROSConfig, OpenCVConfig
from nectar.ai import Detector, DetectionResult
from package_delivery.constants import Config


class Initialize(State):
    def __init__(self, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        blackboard['has_thePkg'] = self.config.has_thePkg
        # Start Simulation Time
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Start time...')
            self.start_time = self.node.get_clock().now()

            blackboard['start_time'] = self.start_time
            yasmin.YASMIN_LOG_INFO('successful Start time!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Start time failed: {e}')
            return ABORT
        
        # Drone
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Drone...')
            if self.config.drone_type == 'mavros':
                drone_config = MavrosConfig(
                    pose_source=PoseSource.VISION,
                    connection_string=self.config.connection_string
                )

            elif self.config.drone_type == 'mavlink':
                drone_config = MavlinkConfig(
                    pose_source=PoseSource.VISION,
                    connection_string=self.config.connection_string
                )
            else:
                yasmin.YASMIN_LOG_ERROR('Invalid drone_type.')
                return ABORT
            if self.config.sim_mode:
                drone = DroneFactory.create("mavros", SITL_GAZEBO_CONFIG)
            else:
                drone = DroneFactory.create(self.config.drone_type, drone_config)

            blackboard['drone'] = drone
            yasmin.YASMIN_LOG_INFO(f'Successful start Drone("{self.config.drone_type}")!')
        
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'DroneFactory failed: {e}')
            return ABORT
        
        # PID controller
        try:
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
            pid_cz = PIDController(
                kp=self.config.z_kp,
                ki=self.config.z_ki,
                kd=self.config.z_kd,
                output_limits=self.config.z_output_lim,
                integral_limits=self.config.z_integral_lim,
            )
            blackboard["pid_cx"] = pid_cx
            blackboard["pid_cy"] = pid_cy
            blackboard["pid_cz"] = pid_cz
            yasmin.YASMIN_LOG_INFO(f'Successful start PID (x, y and z)!')
        
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'PID failed: {e}')
            return ABORT


        # Detector - box
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Detector(box)...')
            self.detector_box = Detector(
                model_source=self.config.box_model_source,
                confidence_threshold=self.config.box_conf,
            )
            yasmin.YASMIN_LOG_INFO('Initializing Detector(box)...')
            self.detector_box = Detector(
                model_source=self.config.box_model_source,
                confidence_threshold=self.config.box_conf,
            )

            yasmin.YASMIN_LOG_INFO('Load Detector(box)...')
            self.detector_box.load()

            blackboard['detector_box'] = self.detector_box
            yasmin.YASMIN_LOG_INFO('Successful start Detector(box)!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Detector(box) failed: {e}')
            return ABORT

        # Camera (Image Handler)
        try:
            yasmin.YASMIN_LOG_INFO('Initializing Camera...')
            if (self.config.sim_mode):
                image_source = self.config.sim_image_source
                cam_config = ROSConfig(
                    topic=self.config.sim_image_source, 
                    compressed=self.config.sim_image_compressed,
                )
            else:
                image_source = self.config.image_source
                cam_config = OpenCVConfig(
                    width=self.config.image_width, 
                    height=self.config.image_height,
                )

            camera = ImageHandler(
                image_source=image_source,
                config=cam_config,
                image_processing_callback=self.detector_box_callback,
            )

            yasmin.YASMIN_LOG_INFO('Open camera...')
            camera.open()

            yasmin.YASMIN_LOG_INFO('Take testing photo...')
            frame_test = camera.take_photo()
            if frame_test is None:
                yasmin.YASMIN_LOG_ERROR("Failed to get frame from camera.")
                return ABORT

            blackboard['camera'] = camera
            yasmin.YASMIN_LOG_INFO('Successful start camera!')
        
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Camera failed: {e}')
            return ABORT 
        return SUCCEED


    def detector_box_callback(self, image : np.ndarray):
        start = datetime.fromtimestamp(self.start_time.nanoseconds / 1e9)
        now = datetime.fromtimestamp(
        self.node.get_clock().now().nanoseconds / 1e9)

        pkg_path = pathlib.Path.home() / 'ros2_ws' / \
            start.strftime('pkgdelivery-%Y-%m-%d_%H-%M-%S')
        raw_path = pkg_path / 'box'
        annotated_path = pkg_path / 'box_annotated'

        raw_file = raw_path / now.strftime('raw-%Y-%m-%d_%H-%M-%S-%f.png')
        annotated_file = annotated_path / \
            now.strftime('annotated-%Y-%m-%d_%H-%M-%S-%f.png')

        os.makedirs(pkg_path, exist_ok=True)
        os.makedirs(raw_path, exist_ok=True)
        os.makedirs(annotated_path, exist_ok=True)

        result = self.detector_box.detect(image, conf=self.config.box_conf)
        result.image = image
        result.annotated_image = self.detector_box.draw_detections(image, result)

        cv2.imwrite(str(raw_file), result.image)
        cv2.imwrite(str(annotated_file), result.annotated_image)
        return result


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

        try:
            drone.takeoff(self.config.takeoff_altitude,max_retries=5, timeout=30.0, precision=0.2)
            yasmin.YASMIN_LOG_INFO(f'Taking off to altitude: {self.config.takeoff_altitude} m...')

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
        drone.delay(1)
        try:
            drone.rtl(altitude=self.config.rtl_altitude, method=RTLMethod.NAVIGATE, land=True)
            yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT