from nectar.vision import(
    ImageHandler,
)

from nectar.control import(
    MavrosDrone,
)

from rclpy.duration import Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT, ABORT
from yasmin_ros.yasmin_node import YasminNode

from precision_landing.constants import(
    SEARCH_TIME,
    FIND_TIME,
    MAX_ALTITUDE,
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

            camera.start() #start the ImageHandler

            start_time = self.node.get_clock().now() #gets the start time of the state
            search_time = Duration(seconds=SEARCH_TIME) #gets the max time in seconds before TIMEOUT

            while (self.node.get_clock().now() - start_time) < search_time: #Executes this sub-state for a max of 2min

                if drone.get_altitude() >= MAX_ALTITUDE: #Aborts if the drone gets past 6m of altitude
                    yasmin.YASMIN_LOG_ERROR('Failed: limit altitude reached.')
                    drone.move_velocity(0.0, 0.0, 0.0, 0.0)
                    drone.delay(1.0)
                    return ABORT

                frame = camera.take_photo()

                #TODO: the drone will be moving in an X shape until an ArUco is in the FOV

            return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"SEARCH SUB-STATE FAILED: {e}")
            return ABORT



class GetTargetBase(State):
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
            camera.start()
            
            while True:
            
                if drone.get_altitude() >= MAX_ALTITUDE: #Aborts if the drone gets past 6m of altitude
                    yasmin.YASMIN_LOG_ERROR('Failed: limit altitude reached.')
                    drone.move_velocity(0.0, 0.0, 0.0, 0.0)
                    drone.delay(1.0)
                    return ABORT
            
                frame = camera.take_photo()

                #TODO: Get the ArUco ID and the base shape and save them into the blackboard
            
            return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"GET_TARGET_BASE SUB-STATE FAILED: {e}")
            return ABORT



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
            camera.start()

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
                    
            return TIMEOUT
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"GET_TARGET_BASE SUB-STATE FAILED: {e}")
            return ABORT