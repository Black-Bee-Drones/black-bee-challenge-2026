import os
import datetime
import cv2

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

import nectar
from nectar.control import (
    DroneFactory,
    MavrosConfig,
    MavlinkConfig,
    PoseSource,
    SITL_GAZEBO_CONFIG,
    MAVLINK_SITL_GAZEBO_CONFIG,
)
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision.camera import ROSConfig
from nectar.ai import Detector

from precision_landing.constants import (
    SIM_MODE,
    CAMERA_SOURCE,
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    DETECTOR_MODEL_SOURCE,
    DETECTOR_CONFIDENCE_THRESHOLD,
)

class Initialize(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.node = YasminNode.get_instance()

        timestamp = self.node.get_clock().now().nanoseconds / 1e9
        now = datetime.datetime.fromtimestamp(timestamp)
        self.photos_folder = now.strftime('precision_landing-%Y-%m-%d-%H-%M')

    def execute(self, blackboard: Blackboard):
        try:
            yasmin.YASMIN_LOG_INFO("Initializing Drone...")

            config = (
                SITL_GAZEBO_CONFIG if SIM_MODE
                else MavrosConfig(pose_source=PoseSource.GPS, expect_lidar=True)
            )
            drone = DroneFactory.create("mavros", config, self.node._executor)
            blackboard["drone"] = drone

            if SIM_MODE:
                cam_config = ROSConfig(topic=CAMERA_SOURCE, compressed=False)
            else:
                cam_config = OpenCVConfig(width=IMAGE_WIDTH, height=IMAGE_HEIGHT)

            camera = ImageHandler(
                image_source=CAMERA_SOURCE,
                config=cam_config,
                image_processing_callback=self.camera_callback,
                #NOTE: There are other ways of doing the continuously image processing,
                #this is one of them, but we can discuss this later
            )

            try:
                self.detector = Detector( #Creates the detector
                    model_source= DETECTOR_MODEL_SOURCE,
                    confidence_threshold= DETECTOR_CONFIDENCE_THRESHOLD,
                )
            
                yasmin.YASMIN_LOG_INFO("Loading the Detector...")
                self.detector.load()
            
                blackboard["detector"] = self.detector
                yasmin.YASMIN_LOG_INFO("Detector succesfully loaded.")
            
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Detector failed: {e}")
                return ABORT
            
            camera.open()

            frame = camera.take_photo()
            if frame is None:
                yasmin.YASMIN_LOG_ERROR("Failed to get frame from camera.")
                return ABORT
            yasmin.YASMIN_LOG_INFO(
                f"Camera ready!"
            )
            blackboard["camera"] = camera

            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Initialization failed: {e}")
            return ABORT

    def camera_callback(self, image): #Runs everytime we call camera.take_photo()
        try:
            os.makedirs(self.photos_folder, exist_ok=True)
            
            timestamp = self.node.get_clock().now().nanoseconds
            
            #os.makedirs(os.path.join(self.photos_folder, 'images'), exist_ok=True)
            #raw_path = os.path.join(self.photos_folder, 'images', f'{timestamp}.png') #Saves the frames in a folder
            #cv2.imwrite(raw_path, image)
            
            result = self.detector.detect(image) #Runs the detector on the frame
            result.image = image
            
            annotated = self.detector.draw_detections(image, result) #Annotates the frames
            #os.makedirs(os.path.join(self.photos_folder, 'annotated'), exist_ok=True)
            #ann_path = os.path.join(self.photos_folder, 'annotated', f'{timestamp}-annotated.png') #Saves the annotaded frames
            #cv2.imwrite(ann_path, annotated)
            #NOTE: We will only use this folders for debugging purposes

            return result
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Detector not working: {e}")
            return image