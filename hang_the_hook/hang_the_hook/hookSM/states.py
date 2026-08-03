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
All the setup can be inherited from followlineSM blackboards
'''
class Align(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

class Descend(State):

    def __init__(self):
        ...

class Hook(State):

    def __init__(self):
        ...

class EndHook(State):

    def __init__(self):
        ...