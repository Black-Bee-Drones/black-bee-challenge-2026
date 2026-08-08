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
from hang_the_hook.utils.check_blackboard import blackboard_check

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
        if not blackboard_check(blackboard=blackboard, args=("drone", "camera")):
            return ABORT

        self.drone = blackboard["drone"]
        self.camera = blackboard["camera"]

        frame = self.camera.take_photo()
        if frame is not None:
            _, _, self.cx, self.cy, self.angle, _, _ = self.hosedetector.detect_line(frame)
            if self.cx == float('nan') or self.cy == float('nan') or self.angle == float('nan'):
                self.hose = (self.cx, self.cy, self.angle)
                blackboard['hose'] = self.hose
                return SUCCEED
            else:
                return ABORT
        else:
            self.altitude = self.drone.get_altitude()
            if self.altitude is not None:
                self.drone.move_to(z= self.altitude + 0.5)
            return ABORT

class Align(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.drone: MavlinkDrone | MavrosDrone
        self.camera: ImageHandler

        self.pid_cx: PIDController
        self.pid_cy: PIDController
        self.pid_angle: PIDController
        self.hose: Tuple[float, float, float]

    def execute(self, blackboard: Blackboard):
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                'drone',
                'camera',
                'pid_cx',
                'pid_cy',
                'pid_angle',
                'hose'
                )
            ):
            return ABORT

        self.drone     = blackboard["drone"]
        self.camera    = blackboard["camera"]
        self.pid_cx    = blackboard["pid_cx"]
        self.pid_cy    = blackboard["pid_cy"]
        self.pid_angle = blackboard["pid_angle"]
        self.hose      = blackboard["hose"]

        self.pid_cx.set_setpoint(self.hose[1])

class Descend(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])


class Hook(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

class EndHook(State):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
