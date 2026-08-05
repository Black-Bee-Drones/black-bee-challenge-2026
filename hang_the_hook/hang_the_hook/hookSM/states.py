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
        self.altitude: float
        self.camera: ImageHandler

        self.hosedetector = LineDetector(color="red", estimation_method=RotatedRect(), color_space=ColorSpace.HSV)
        self.hose: Tuple[float, float, float]
        self.cx: float
        self.cy: float
        self.angle: float

        self.pid_cx = PIDController(KP, KI,KD, output_deadband= PID_DEADBAND)
        self.pid_cy = PIDController(KP, KI,KD, output_deadband= PID_DEADBAND)
        self.pid_angle = PIDController(KP, KI,KD, output_deadband= PID_DEADBAND)

        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        self.drone, self.camera = blackboard["drone"], blackboard["camera"]

        blackboard["pid_cx"], blackboard["pid_cy"], blackboard["pid_angle"] = \
            self.pid_cx, self.pid_cy, self.pid_angle

        frame = self.camera.take_photo()
        if frame is not None:
            _, _, self.cx, self.cy, self.angle, _, _ = self.hosedetector.detect_line(frame)
            self.hose = (self.cx, self.cy, self.angle)
            return SUCCEED
        else:
            self.altitude = self.drone.get_altitude()
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
