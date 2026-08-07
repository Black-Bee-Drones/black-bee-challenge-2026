from traceback import print_exc

import math
import nectar
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision import LineDetector, RotatedRect, ColorSpace
from nectar.control import PIDController, AltitudeSource, MavrosDrone
import cv2
from nectar.control import DroneFactory, MavrosConfig, PoseSource
from nectar.control.types import MoveReference

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from datetime import datetime

from followlineSM.constants import (
    CENTER_VARIATION,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    IMAGE_SOURCE,
)

from hang_the_hook.followlineSM.constants import (
    CENTER_VARIATION,
    HOSE_COUNTER,
    FRAMES_TO_CONFIRM_HOSE
)

from nectar.vision import(
    ImageHandler,
    OpenCVConfig,
    LineDetector,
    RotatedRect,
    ColorSpace,
)

from nectar.control import(
    PIDController,
    AltitudeSource,
    MavrosDrone,
    MavlinkDrone,
    MoveReference
)

from hang_the_hook.utils.line_follow import segue_linha
from cv2 import imwrite as cv2_imwrite

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from datetime import datetime

    MIN_BLUE_FRAMES,
    MIN_RED_FRAMES,
    FOWARD_SPEED_BLUE_LINE,
)

class SetupLineDetection(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
            line_detector = LineDetector(color="blue", estimation_method=RotatedRect(), color_space=ColorSpace.HSV)
            blackboard["line_detect"] = line_detector

            hose_detector = LineDetector(color="red", estimation_method=RotatedRect(), color_space=ColorSpace.HSV)
            blackboard["hose_detect"] = hose_detector

            camera: ImageHandler = blackboard["camera"]

            camera.image_processing_callback = lambda frame: line_detector.detect_line(frame)

            blackboard["hose_counter"] = HOSE_COUNTER

            return SUCCEED

        except Exception as e:
            print(f"Setup failed: {e}")
            print_exc()

            return ABORT

class SearchBlueLine(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
            # Retrieve the line linedetector and image handler from the Blackboard
            linedetector = blackboard["line_detect"]
            camera: ImageHandler = blackboard["camera"]
            counterblue = 0
            oldcxb = 0
            oldcyb = 0

            if not linedetector or not camera:
                print("One or more detectors or image handler not initialized.")
                return ABORT

            # Start the image handler to process frames and detect lines
            camera.start()

            # Main loop for line following
            while True:
                frame = camera.take_photo()
                resultb, _, cxb, cyb, angleb, wb, hb = linedetector.detect_line(frame, draw=True)

                #Skips if no line detected
                if not resultb or cxb is None or cyb is None:
                    continue

                distanceb = math_dist((cxb,cyb),(oldcxb,oldcyb))

                if (distanceb < CENTER_VARIATION):
                    counterblue = counterblue + 1

                if counterblue == MIN_BLUE_FRAMES:
                    counterblue = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2_imwrite(f"../images/{now}.png", resultb)
                    blackboard["angle_blue"] = angleb
                    blackboard["width_blue"] = wb
                    blackboard["height_blue"] = hb
                    break

                oldcxb = cxb
                oldcyb = cyb

            return SUCCEED
        except Exception as e:
            print(f"Blue line searching failed: {e}")
            return ABORT
        finally:
            camera.stop()

class FollowBlueLine(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

        self.pid_cx = PIDController(
            kp=CX_KP, 
            ki=CX_KI, 
            kd=CX_KD, 
            setpoint = FRAME_WIDTH // 2
            output_limits=(-1.0, 1.0)
            integral_limits=(-0.5, 0.5)
            output_deadband=0.0
        )
                                                                                       
        self.pid_angle = PIDController(
            kp=ANGLE_KP, 
            ki=ANGLE_KI, 
            kd=ANGLE_KD, 
            setpoint= 0.0
            output_limits=(-1.0, 1.0)
            integral_limits=(-0.5, 0.5)
            output_deadband=0.0
        )
        
        self.hosedetector = None


    def execute(self, blackboard: Blackboard):
        try:
            pid_cx: PIDController = blackboard["pid_cx"]
            pid_angle: PIDController = blackboard["pid_angle"]
            camera: ImageHandler = blackboard["camera"]
            drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
            linedetector: LineDetector = blackboard["line_detect"]
            hosedetector: LineDetector = blackboard["hose_detect"]
            hose_counter = blackboard["hose_counter"]
            
            if not camera:
                print("Image handler not initialized.")
                return ABORT

            camera.start()

            while True:
                frame = camera.take_photo()
                resultb, _, cx, cy, angle, w, h = linedetector.detect_line(frame, draw=True)

                if cx is None:
                    # perdeu a linha -> volta pra SEARCH_BLUE_LINE
                    drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, reference=MoveReference.BODY)
                    return ABORT

                #TODO: Logic PID
                vx = pid_cx.update(cx)
                vyaw = pid_angle.update(angle)
                drone.move_velocity(vx=vx, vy=0.0, vz=0.0, vyaw=vyaw, reference=MoveReference.BODY)

                
                _, _, hose_cx, _, _, _, _ = hosedetector.detect_line(frame, draw=True)

                if hose_cx is not None:
                        hose_counter += 1
                else:
                    hose_counter = 0

                
                blackboard["hose_counter"] = hose_counter
                
                if hose_counter >= FRAMES_TO_CONFIRM_HOSE:
                    print("Hose detected! Stopping line following.")
                    drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, reference=MoveReference.BODY)
                    return SUCCEED

        except Exception as e:
            print(f"Follow blue line failed: {e}")
            try:
                drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, reference=MoveReference.BODY)
            except Exception:
                pass
            return ABORT
