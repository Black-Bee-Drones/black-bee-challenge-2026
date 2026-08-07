import math
import nectar
from typing import Tuple
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision import LineDetector, RotatedRect, ColorSpace
from nectar.control import PIDController, AltitudeSource, MavrosDrone, MavlinkDrone

import cv2

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from hang_the_hook.core.constants import IMAGE_SOURCE
from hang_the_hook.hookSM.constants import(
    KP,
    KI,
    KD,
    PID_DEADBAND
)

'''
Initial cx, cy and angle of the red hose can be inherited from FollowLineSM
State FindRedLine() stands only for testing
'''
class FindRedLine(State):

    def __init__(self) -> None:
        super().__init__(outcomes=[SUCCEED,ABORT])

        self.drone: MavrosDrone | MavlinkDrone
        self.altitude: float | None
        self.camera: ImageHandler

        self.hosedetector: LineDetector
        self.hose: Tuple[float, float, float]

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        self.drone = blackboard["drone"]
        self.camera = blackboard["camera"]

        frame = self.camera.take_photo()
        if frame is not None:
            _, _, self.cx, self.cy, self.angle, _, _ = self.hosedetector.detect_line(frame)
            self.hose = (self.cx, self.cy, self.angle)
            return SUCCEED
        else:
            self.altitude = self.drone.get_altitude()
            if self.altitude is not None:
                self.drone.move_to(z= self.altitude + 0.5)
            return ABORT



class Align(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

class Descend(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])


class Hook(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

class EndHook(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
