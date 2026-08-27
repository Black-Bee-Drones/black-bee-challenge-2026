import cv2

from nectar.vision import(
    ImageHandler,
    Aruco,
)
from nectar.control import(
    MavrosDrone,
    MavlinkDrone,
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
            drone.delay(0.5)

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

                for _ in range(2): #Repeats the detection 2 times for security
                    frame = camera.take_photo()
                    bbox, aruco_id = aruco.detect(frame.image, draw=True)
                    
                    if aruco_id is not None:
                        blackboard["aruco_id"] = str(aruco_id) #The index for the numbers are 0, 1 and 2
                        drone.move_velocity(vx=0.0, vy=0.0, vz=0.0)
                        yasmin.YASMIN_LOG_INFO("Detected the ArUco, moving closer... ")
                        yasmin.YASMIN_LOG_INFO(f"ARUCO ID: {aruco_id}")
                        yasmin.YASMIN_LOG_INFO(f"Bbox of ARUCO: {bbox}")
                    
                        yaw_angle = aruco.calculateYawFromCorners(bbox=bbox)

                        try:
                            aruco_shape = self.get_aruco_shape(frame, bbox)
                        except Exception as i:
                            yasmin.YASMIN_LOG_INFO(f"DIDN'T DETECT shape..., retaking photo")
                            yasmin.YASMIN_LOG_INFO(f"{i}")
                            drone.move_to(yaw=yaw_angle)
                            drone.move_to(x=1, reference=MoveReference.BODY)
                            frame = camera.take_photo()
                            bbox2, id = aruco.detect(frame.image)

                            aruco_shape = self.get_aruco_shape(frame, bbox2)

                        blackboard["aruco_shape"] = aruco_shape
                        yasmin.YASMIN_LOG_INFO(f"Aruco shape detected: {aruco_shape}")
                    
                        return SUCCEED

                if idx < len(WAYPOINTS):
                    x, y = WAYPOINTS[idx]
                    drone.move_to(x=x, y=y, z=0, reference=MoveReference.TAKEOFF)
                    idx += 1
                    drone.delay(0.5)
                else:
                    yasmin.YASMIN_LOG_INFO("FAILED, didn't find the ArUco")
                    return FAIL

            return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"SEARCH SUB-STATE FAILED: {e}")
            return ABORT

    def get_aruco_shape(self, frame, bbox): #Function that gets the shape around the ArUco
        aruco_shape = None
        for s in frame.filter_by_class(['Triangle', 'Hexagon', 'Star']):
            if(abs(self.bbox_center(bbox)[0]-s.center[0])<=s.width/2):
                aruco_shape = s.class_name

        if aruco_shape is not None:
            return str(aruco_shape)
        else:
            return None

    def bbox_center(self, bbox): #Function to get the center of the ArUco by its bbox
        corners = bbox[0][0]  # unwrap: tupla -> array (1,4,2) -> array (4,2)
    
        sup_left = corners[0]
        inf_right = corners[2]

        cx = (sup_left[0] + inf_right[0]) / 2
        cy = (sup_left[1] + inf_right[1]) / 2
        center = (cx, cy)
        return center



class FindTargetBase(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, TIMEOUT])
        self.node = YasminNode.get_instance()
    
    def execute(self, blackboard: Blackboard):
        try:
            drone: MavrosDrone = blackboard["drone"]
                    
            camera: ImageHandler = blackboard["camera"]
                                        
            if not camera:
                yasmin.YASMIN_LOG_ERROR("Camera or ImageHandler not initialized")
                return ABORT

            start_time = self.node.get_clock().now() #gets the start time of the state
            find_time = Duration(seconds=FIND_TIME) #gets the max time in seconds before TIMEOUT

            idx = 0

            drone.move_to(x=0.0, y=0.0, reference=MoveReference.TAKEOFF)
            
            while (self.node.get_clock().now() - start_time) < find_time:
                    
                if drone.get_altitude() >= MAX_ALTITUDE: #Aborts if the drone gets past 6m of altitude
                    yasmin.YASMIN_LOG_ERROR('Failed: limit altitude reached.')
                    drone.move_velocity(0.0, 0.0, 0.0, 0.0)
                    drone.delay(1.0)
                    return ABORT
                    
                for _ in range(2):
                    frame = camera.take_photo()

                    #NOTE: Also incertain about this one, need to test
                    for s in frame.filter_by_class([blackboard["aruco_shape"]]):
                        for n in frame.filter_by_class([blackboard["aruco_id"]]):
                            if (abs(n.center[0] - s.center[0]) <= s.width/2) and (abs(n.center[1] - s.center[1]) <= s.height/2):
                                #Verify if there's a base with the aruco_id inside the aruco_shape we want
                                yasmin.YASMIN_LOG_INFO("DETECTED TARGET BASE")
                                return SUCCEED

                if idx < len(WAYPOINTS):
                    x, y = WAYPOINTS[idx]
                    drone.move_to(x=x, y=y, z=0, reference=MoveReference.TAKEOFF)
                    idx += 1
                    drone.delay(0.5)
                else:
                    yasmin.YASMIN_LOG_INFO("FAILED, didn't find the equivalent base")
                    return FAIL
                    
            return TIMEOUT
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"GET_TARGET_BASE SUB-STATE FAILED: {e}")
            return ABORT