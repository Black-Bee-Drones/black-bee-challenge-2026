import os
import time
import cv2

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

from dart import (
    DART,
    PredictResult,
    Detection,
)

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

        if "model" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("DART model not available.")
            return ABORT
        model: DART = blackboard["model"]

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

        drone.move_velocity(0.0, 0.0, 0.0)
        drone.move_to(0.0, 0.0, self.config.safe_altitude)
        drone.delay(2)

        try:
            lost = 0
            while not self.timed_out():
                frame = camera.take_photo()
                if frame is None:
                    yasmin.YASMIN_LOG_WARN("Failed to get frame from camera, skipping cycle.")
                    drone.delay(0.1)
                    continue

                results = model.predict(frame, self.config.conf_threshold)
                target_found = bool(results) and bool(results[0].detections)

                result: PredictResult = results[0]
                detection: Detection = result.detections[0]
                self.save_detections(frame, detection, self.config.photos_folder)
                x1, y1, x2, y2 = detection.box_xyxy
                target_x = (x1 + x2) // 2
                target_y = (y1 + y2) // 2

                if not target_found:
                    lost += 1
                    yasmin.YASMIN_LOG_WARN(f"Box not detected ({lost}/{self.config.lost_tolerance})...")
                    drone.move_to(x=0.0, y=0.0, z=0.0)  
                    if lost >= self.config.lost_tolerance:
                        yasmin.YASMIN_LOG_ERROR("Lost detection exceeded: restarting state...")
                        return FAIL
                    drone.delay(0.2)
                    continue
                
                # result: DetectionResult = detector_box.detect(frame, conf=self.config.box_conf)
                # detections = result.filter_by_class([self.config.box_class_name])
                
                # # tolerates some missed detections before giving up, until the timeout
                # if not detections:
                #     lost += 1
                #     yasmin.YASMIN_LOG_WARN(f"Box not detected ({lost}/{self.config.lost_tolerance}) Holding position...")
                #     drone.move_velocity(0.0, 0.0, 0.0)
 
                #     if lost >= self.config.lost_tolerance:
                #         yasmin.YASMIN_LOG_ERROR("Lost detection exceeded, aborting approach.")
                #         return FAIL
                #     continue
 
                # lost = 0
                # best_det = max(detections, key=lambda d: d.confidence)
 
                error_x_px = self.config.image_width // 2 - target_x
                error_y_px = self.config.image_height // 2 - target_y
 
                altitude = drone.get_altitude()
 
                error_x = self.ppm(error_x_px, altitude, 86, self.config.image_width)
                error_y = self.ppm(error_y_px, altitude, 47, self.config.image_height)
                error_z = self.config.dropoff_altitude - altitude
 
                vx = pid_cy.update(error_y)
                vy = pid_cx.update(-error_x)
 
                aligned = max(abs(error_x), abs(error_y)) < self.config.approach_tolerance
                vz = pid_cz.update(error_z) if aligned else 0.0
 
                drone.move_velocity(vx, vy, vz)

                yasmin.YASMIN_LOG_INFO(
                    f"Detection at: error_x={error_x:.2f}, error_y={error_y:.2f}, "
                    f"error_z={error_z:.2f} | vx={vx:.2f}, vy={vy:.2f}, vz={vz:.2f} | alt={altitude:.2f}"
                )

                drone.delay(0.1)

                if aligned and abs(error_z) < self.config.dropoff_tolerance:
                    yasmin.YASMIN_LOG_INFO("Approaching succeeded! Delivering package...")
                    pid_cx.reset(); pid_cy.reset(); pid_cz.reset()
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

    def save_detections(self, frame, detection: Detection, save_dir: str) -> None:
        # nao sei se ta certo, tentei fazer com base no codigo do roncas da sae
        os.makedirs(os.path.join(save_dir, "images"), exist_ok=True)
        os.makedirs(os.path.join(save_dir, "labels"), exist_ok=True)
        image_height, image_width = frame.shape[:2]
        annotated = frame
    
        name = f"{int(time.time() * 1000)}"
        image_path = os.path.join(save_dir, "images", f"{name}.jpg")
        label_path = os.path.join(save_dir, "labels", f"{name}.jpg")
    
        x1, y1, x2, y2 = detection.box_xyxy
        cv2.rectangle(annotated, (x1, y1), (x2, y2), 1)
    
        cv2.imwrite(image_path, frame)
        cv2.imwrite(label_path, annotated)

        return