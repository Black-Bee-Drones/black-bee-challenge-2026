from rclpy.time import Time, Duration
import math

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

import nectar
from nectar.control import (
    MavrosDrone,
    MavlinkDrone,
    PIDController,
)
from nectar.vision import (
    ImageHandler,
)
from nectar.ai import DetectionResult

from package_delivery.constants import Config


class Approach(State):
    def __init__(self, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT, TIMEOUT, FAIL])
        self.config = config
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        self.mission_start_time = blackboard.get("start_time")
        self.state_start_time = self.node.get_clock().now()
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT 
        
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')
        
        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
                
        if "camera" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Camera not available.")
            return ABORT
        camera: ImageHandler = blackboard["camera"]

        # if "detector_box" not in blackboard:
        #     yasmin.YASMIN_LOG_ERROR("Detector(box) not available.")
        #     return ABORT
        # detector_box: Detector = blackboard["detector_box"]

        if "pid_cx" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("X PID Controller not available.")
            return ABORT
        if "pid_cy" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Y PID Controller not available.")
            return ABORT
        if "pid_cz" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Z PID Controller not available.")
            return ABORT
        
        pid_cx: PIDController = blackboard["pid_cx"]
        pid_cy: PIDController = blackboard["pid_cy"]
        pid_cz: PIDController = blackboard["pid_cz"]
        
        pid_cx.reset()
        pid_cy.reset()
        pid_cz.reset()

        pid_cx.set_setpoint(0.0)
        pid_cy.set_setpoint(0.0)
        pid_cz.set_setpoint(0.0)
        
        drone.delay(1)

        try:
            lost = 0
            while not self.timed_out():
                # frame = camera.take_photo()
                # if frame is None:
                #     yasmin.YASMIN_LOG_WARN("Failed to get frame from camera, skipping cycle.")
                #     continue
 
                result: DetectionResult = camera.take_photo()
                if result is None:
                    yasmin.YASMIN_LOG_WARN("Failed to get frame from camera, skipping cycle")
                    continue
                detections = result.filter_by_class([self.config.box_class_name])
                
                # tolerates some missed detections before giving up, until the timeout
                if not detections:
                    lost += 1
                    yasmin.YASMIN_LOG_WARN(f"Box not detected ({lost}/{self.config.lost_tolerance}) Holding position...")
                    drone.move_velocity(0.0, 0.0, 0.0)
 
                    if lost >= self.config.lost_tolerance:
                        yasmin.YASMIN_LOG_ERROR("Lost detection exceeded, aborting approach.")
                        return FAIL
                    continue
 
                lost = 0
                best_det = max(detections, key=lambda d: d.confidence)
                target_x, target_y = best_det.center
 
                error_x_px = self.config.image_width // 2 - target_x
                error_y_px = self.config.image_height // 2 - target_y
 
                altitude = drone.get_altitude()
 
                error_x = self.ppm(error_x_px, altitude, 86, self.config.image_width)
                error_y = self.ppm(error_y_px, altitude, 47, self.config.image_height)
                error_z = altitude - self.config.dropoff_altitude
 
                vx = pid_cy.update(error_y)
                vy = pid_cx.update(-error_x)
 
                if max(abs(error_x), abs(error_y)) < self.config.approach_tolerance:
                    vz = pid_cz.update(error_z)
                else:
                    vz = 0.0
 
                drone.move_velocity(vx, vy, vz)
 
                if abs(error_z) < self.config.dropoff_tolerance:
                    yasmin.YASMIN_LOG_INFO("Approaching succeeded! Delivering package...")
                    return SUCCEED
                
            yasmin.YASMIN_LOG_ERROR("Approaching box timed out.")
            return TIMEOUT
                
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Approaching box failed: {e}")
            return ABORT

    def ppm(self, delta_pixel: int, altitude: float, fov_degrees: float, frame_px: int) -> float:
        angle_rad = math.radians(fov_degrees) / 2
        ratio = (math.tan(angle_rad) * altitude) / (frame_px // 2)

        return delta_pixel * ratio
    
    def timed_out(self) -> bool:
        now = self.node.get_clock().now()
        if self.mission_start_time is not None and hasattr(self.config, "mission_timeout"):
            if now - self.mission_start_time > Duration(seconds=self.config.mission_timeout):
                return True
        return now - self.state_start_time > Duration(seconds=self.config.approach_timeout)
    