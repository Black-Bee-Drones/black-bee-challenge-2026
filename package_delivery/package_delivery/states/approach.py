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
        
        # drone.delay(1)

        try:
            lost : int = 0
            aligned_frames : int = 0
            while not self.timed_out():
                
                result: DetectionResult = camera.take_photo()
                # yasmin.YASMIN_LOG_INFO(result)
                
                if result is None:
                    yasmin.YASMIN_LOG_WARN("Failed to get frame from camera, skipping cycle")
                    continue
                
                detections = result.filter_by_class([self.config.box_class_name])
                
                
                # Box Not Detect!
                if not detections:
                    lost += 1
                    aligned_frames = 0
                    yasmin.YASMIN_LOG_WARN(f"Box not detected ({lost}/{self.config.lost_tolerance}) Holding position...")
                    
                    drone.move_velocity(0.0, 0.0, 0.0)
 
                    if lost >= self.config.lost_tolerance:
                        yasmin.YASMIN_LOG_WARN("Detection lost. Increasing altitude to restart search...")
                        if drone.get_altitude() < self.config.max_altitude:
                            drone.move_to(z=0.8)
                        else:
                            yasmin.YASMIN_LOG_WARN("Detection lost. Max altitude reached, decreasing 20 cm...")
                            drone.move_to(z=-0.8)
                        pid_cx.reset()
                        pid_cy.reset()
                        pid_cz.reset()
                        yasmin.YASMIN_LOG_INFO("Restarting box detection")
                        lost = 0
                        aligned_frames = 0
                    continue

                # BOX DETECT
                lost = 0
                best_det = max(detections, key=lambda d: d.confidence)
                target_x, target_y = best_det.center
 
                error_x_px = target_x - self.config.image_width // 2
                error_y_px = target_y - self.config.image_height // 2
 
                altitude = drone.get_altitude()
 
                error_x = self.ppm(error_x_px, altitude, 60, self.config.image_width)
                error_y = self.ppm(error_y_px, altitude, 47, self.config.image_height) + self.config.claw_offset
                error_z = altitude - self.config.dropoff_altitude

                aligned = max(abs(error_x), abs(error_y)) <= self.config.approach_tolerance
                px_aligned = max(abs(error_x_px), abs(error_y_px)) <= self.config.approach_tolerance_px

                vx = pid_cy.update(error_y)
                vy = pid_cx.update(error_x)
                vz = pid_cz.update(error_z) if px_aligned else 0.0

                if aligned:
                    aligned_frames += 1
                    yasmin.YASMIN_LOG_INFO(
                        f"Box aligned "
                        f"({aligned_frames}/"
                        f"{self.config.required_frames})"
                    )
                else:
                    aligned_frames = 0

                drone.move_velocity(vx, vy, vz)
 
                if (abs(error_z) <= 0) and aligned_frames >= self.config.required_frames:
                    drone.move_velocity(0.0, 0.0, 0.0)
                    yasmin.YASMIN_LOG_INFO(f"Approach confirmed with {aligned_frames} consecutive aligned frames. Stopping drone before delivery.")
                    return SUCCEED

            yasmin.YASMIN_LOG_ERROR("Approaching box timed out.")
            return ABORT
          
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
