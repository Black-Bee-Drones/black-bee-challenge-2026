import math
import nectar
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision import LineDetector, RotatedRect, ColorSpace
from nectar.control import PIDController, AltitudeSource, MavrosDrone

import cv2

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode


'''
Initial cx, cy and angle of the red hose can be inherited from FollowLineSM
State FindRedLine() stands only for testing
'''
class FindRedLine(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED,ABORT])

    def execute(self, blackboard: Blackboard) -> str:
        return super().execute(blackboard)

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
