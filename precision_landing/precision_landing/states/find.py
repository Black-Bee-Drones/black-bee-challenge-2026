import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT
from yasmin_ros.yasmin_node import YasminNode

from rclpy.duration import Duration

import nectar
from nectar.control import MavrosDrone
from nectar.vision import ImageHandler

from precision_landing.constants import (
    SEARCH_TIME,
)

class Find(State):
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
            
            camera.start() #Start the ImageHandler

            start_time = self.node.get_clock().now() #gets the start time of the state
            time = Duration(seconds=SEARCH_TIME) #gets the max time in seconds before TIMEOUT

            while (self.node.get_clock().now() - start_time) < time: #Executes the State for a max of 3min and 30sec
                frame = camera.take_photo()

                target_base_number, target_base_shape = self.get_target_base(frame)
                #saves the number and shape of the target base

                while target_base_number is None:
                    drone.move_to() #TODO: move the drone in an X shape across the arena to search for the ArUco

                blackboard["target_base_number"] = target_base_number
                blackboard["target_base_shape"] = target_base_shape

            return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"FIND STATE FAILED: {e}")
            return ABORT

        finally:
            camera.stop()

    def get_target_base(self, img): #TODO: Has to return the ID of the ArUco and the shape of that base as Strings
        number = self.get_aruco_id(img)
        shape = self.get_target_base_shape(img)
        return str(number), shape

    def get_aruco_id(self, image): #TODO: Has to return the ID of the ArUco
        pass

    def get_target_base_shape(self, image): #TODO: Has to return the shape of the ArUco base
        pass