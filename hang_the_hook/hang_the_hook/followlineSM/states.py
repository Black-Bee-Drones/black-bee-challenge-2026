import math
import nectar
from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision import LineDetector, RotatedRect, ColorSpace
from nectar.control import PIDController, AltitudeSource, MavrosDrone
from line_follow import segue_linha
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
    ANGLE_KD,
    ANGLE_KI,
    ANGLE_KP,
    CX_KD,
    CX_KI,
    CX_KP,
    FRAME_WIDTH,
    MIN_BLUE_FRAMES,
    MIN_RED_FRAMES,
    FOWARD_SPEED_BLUE_LINE,
)

class SetupLineDetection(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.linedetector: LineDetector
        self.hosedetector: LineDetector
        self.handler: ImageHandler
        self.drone: MavrosDrone | MavlinkDrone
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
            # Initialize the line linedetector with the desired color and color space
            # LineDetector runs @staticmethods at estimation_method, so parameterize the class or a object of it have no difference at all
            # However, IntelliSense becomes a nuisance if you don't parameterize the instance
            self.linedetector = LineDetector(color="blue", estimation_method=RotatedRect(), color_space=ColorSpace.HSV)
            self.hosedetector = LineDetector(color="red", estimation_method=RotatedRect(), color_space=ColorSpace.HSV)

            # blackboard is shared across submachines, so there is no need to instantiate a new drone
            self.drone = blackboard["drone"]

            # Set up the image handler with IMAGE_SOURCE as the source
            self.handler = ImageHandler(
                image_source=IMAGE_SOURCE,
                config=OpenCVConfig(width=1280, height=720),
                image_processing_callback=lambda frame: self.linedetector.detect_line(frame),
                show_result="Camera View",
            )

            # Store the linedetector and handler in the Blackboard for later use
            blackboard["line_detector"] = self.linedetector
            blackboard["hose_detector"] = self.hosedetector
            blackboard["image_handler"] = self.handler

            return SUCCEED

        except Exception as e:
            print(f"Setup failed: {e}")
            return ABORT

class SearchBlueLine(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
            # Retrieve the line linedetector and image handler from the Blackboard
            linedetector = blackboard["line_linedetector"]
            handler = blackboard["image_handler"]
            counterblue = 0
            oldcxb = 0
            oldcyb = 0

            if not linedetector or not handler:
                print("One or more detectors or image handler not initialized.")
                return ABORT

            # Start the image handler to process frames and detect lines
            handler.start()

            # Main loop for line following
            while True:
                frame = handler.take_photo()
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
            handler.stop()

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
            self.hosedetector = Blackboard["hose_detector"]
            handler = Blackboard["image_handler"]
            drone = Blackboard["drone"]
            counterred = 0
            oldcxr = 0
            oldcyr = 0

            if not handler:
                print("Image handler not initialized.")
                return ABORT

            while True:
                frame = handler.take_photo()
                cx, cy, angle = segue_linha(frame)

                if cx is None:
                    # perdeu a linha -> volta pra SEARCH_BLUE_LINE
                    return SUCCEED

                #TODO: Logic PID
                
                vx = self.pid_cx.update(cx)      
                vyaw = self.pid_angle.update(angle)
                
                drone.move_velocity(vx=vx, vy=FOWARD_SPEED_BLUE_LINE, vz=0.0, vyaw=vyaw, reference=MoveReference.BODY)
                
                result, _, newcxh, newcyh, _, _, _ = self.hosedetector.detect_line(frame, draw=True)
                dist = math.dist((newcxh,newcyh),(oldcxr,oldcyr))
                if (dist < CENTER_VARIATION):
                    counterred = counterred + 1
                if counterred == MIN_RED_FRAMES:
                    counterred = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"../images/{now}.png", result)
                    break
                
            return SUCCEED
        except Exception as e:
            print(f"Follow blue line failed: {e}")
            return ABORT
        finally:
            handler.stop()
