import math
import cv2

from nectar.vision import(
    ImageHandler,
    Aruco,
)
from nectar.control import(
    MavrosDrone,
    MoveReference,
)
from nectar.ai import Detector

from rclpy.duration import Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT, ABORT
from yasmin_ros.yasmin_node import YasminNode

from precision_landing.constants import(
    SEARCH_TIME,
    FIND_TIME,
    MAX_ALTITUDE,
    MARKER_DICT,
    ARUCO_SIZE,
    WAYPOINTS,
)



class Search(State): #Sub-state that will only move around the arena until it detects an ArUco
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, TIMEOUT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
            blackboard["use_detector"] = False #To prevent we dont run the detector when we don't need

            if "drone" not in blackboard: #IFs que conferem se está tudo certo antes de iniciar o estado
                yasmin.YASMIN_LOG_ERROR("Drone not Available... ")
                return ABORT
            drone: MavrosDrone = blackboard["drone"]
                        
            if "camera" not in blackboard:
                yasmin.YASMIN_LOG_ERROR("Camera not Available... ")
                return ABORT
            camera: ImageHandler = blackboard["camera"]
            
            if not camera:
                yasmin.YASMIN_LOG_ERROR("Camera or ImageHandler not initialized")
                return ABORT
            camera.open()
            camera.run()

            aruco = Aruco(marker_dict=MARKER_DICT, tag_size=ARUCO_SIZE)

            start_time = self.node.get_clock().now() #gets the start time of the state
            search_time = Duration(seconds=SEARCH_TIME) #gets the max time in seconds before TIMEOUT
            idx = 0

            while (self.node.get_clock().now() - start_time) < search_time: #Executes this sub-state for a max of 2min

                if drone.get_altitude() >= MAX_ALTITUDE: #Aborts if the drone gets past 6m of altitude
                    yasmin.YASMIN_LOG_ERROR('Failed: limit altitude reached.')
                    drone.move_velocity(0.0, 0.0, 0.0, 0.0)
                    drone.delay(1.0)
                    return ABORT

                frame = camera.take_photo()
                bbox, aruco_id = aruco.detect(frame, draw=True)

                if aruco_id is not None:
                    blackboard["Aruco_ID"] = aruco_id
                    blackboard["Bbox"] = bbox
                    drone.move_velocity(vx=0.0, vy=0.0, vz=0.0)
                    yasmin.YASMIN_LOG_INFO("Detected the ArUco, moving closer... ")
                    yasmin.YASMIN_LOG_INFO(f"ARUCO ID: {aruco_id}")
                    yasmin.YASMIN_LOG_INFO(f"Bbox of ARUCO: {bbox}")

                    yaw_angle = aruco.calculateYawFromCorners(bbox=bbox)
                    drone.move_to(yaw=yaw_angle)
                    drone.move_to(x=1.5, MoveReference = MoveReference.BODY) # Moves the drone a little bit closer to the aruco
                    blackboard["use_detector"] = True
                    frame = camera.take_photo()

                    blackboard["aruco_shape"] = self.get_aruco_shape(frame, bbox)
                    #TODO: detect the aruco base shape with YOLO

                    return SUCCEED

                if idx < len(WAYPOINTS):
                    x, y = WAYPOINTS[idx]
                    drone.move_to(x=x, y=y, z=0, reference=MoveReference.TAKEOFF)
                    idx += 1
                else:
                    yasmin.YASMIN_LOG_INFO("FAILED, didn't find the ArUco")
                    return FAIL

            return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"SEARCH SUB-STATE FAILED: {e}")
            return ABORT

    def get_aruco_shape(self, frame, bbox): #TODO: function to detect the shape around the aruco
        pass



class FindTargetBase(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, TIMEOUT])
        self.node = YasminNode.get_instance()
    
    def execute(self, blackboard: Blackboard):
        try:
            blackboard["use_detector"] = True

            drone: MavrosDrone = blackboard["drone"]
                    
            camera: ImageHandler = blackboard["camera"]
                                        
            if not camera:
                yasmin.YASMIN_LOG_ERROR("Camera or ImageHandler not initialized")
                return ABORT

            camera.open()
            camera.run()

            start_time = self.node.get_clock().now() #gets the start time of the state
            find_time = Duration(seconds=FIND_TIME) #gets the max time in seconds before TIMEOUT
            
            while (self.node.get_clock().now() - start_time) < find_time:
                    
                if drone.get_altitude() >= MAX_ALTITUDE: #Aborts if the drone gets past 6m of altitude
                    yasmin.YASMIN_LOG_ERROR('Failed: limit altitude reached.')
                    drone.move_velocity(0.0, 0.0, 0.0, 0.0)
                    drone.delay(1.0)
                    return ABORT
                    
                frame = camera.take_photo()
        
                #TODO: Detect if there's an equivalent base with the ID and shape we saved before
                #NOTE: Use Lipedras' DART for the detection for this sub-state
                    
            return TIMEOUT
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"GET_TARGET_BASE SUB-STATE FAILED: {e}")
            return ABORT