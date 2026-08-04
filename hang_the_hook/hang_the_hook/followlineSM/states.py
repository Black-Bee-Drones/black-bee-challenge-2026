import math
from hang_the_hook.core.constants import(
    ANGLE_KD,
    ANGLE_KI,
    ANGLE_KP,
    CX_KD,
    CX_KI,
    CX_KP,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    IMAGE_SOURCE
)

from hang_the_hook.followlineSM.constants import (
    CENTER_VARIATION
)

from nectar.vision import ImageHandler, OpenCVConfig
from nectar.vision import LineDetector, RotatedRect, ColorSpace
from nectar.control import PIDController, AltitudeSource, MavrosDrone, MavlinkDrone
from line_follow import segue_linha
import cv2
from nectar.control import DroneFactory, MavrosConfig, PoseSource
from nectar.control.types import MoveReference

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from datetime import datetime


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

                distanceb = math.dist((cxb,cyb),(oldcxb,oldcyb))

                if (distanceb < CENTER_VARIATION):
                    counterblue = counterblue + 1

                if counterblue == 5:
                    counterblue = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"../images/{now}.png", resultb)
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

        self.pid_cx = PIDController(kp=CX_KP, ki=CX_KI, kd=CX_KD, setpoint = FRAME_WIDTH // 2)
        self.pid_angle = PIDController(kp=ANGLE_KP, ki=ANGLE_KI, kd=ANGLE_KD, setpoint= 0.0)


    def execute(self, blackboard: Blackboard):
        try:
            handler = blackboard["image_handler"]
            drone = blackboard["drone"]

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
                '''
                vx = self.pid_cx.update(cx)
                vyaw = self.pid_angle.update(angle)

                ^^^^^^^^^^^^ essas funções já fizeram a conversão pro ideal (meio da imagem e 0 graus, que é o setpoint), falta transformar essas variáveis de velocidade em movimentação no drone.

                drone.move_velocity(vx=vx, vy=0.0, vz=0.0, vyaw=vyaw, reference=MoveReference.BODY)
                ^^^^^algo assim, eu acho [a gente tem o retorno de cy tbm, talvez dê pra fazer algo com ele]
                '''


        except Exception as e:
            print(f"Follow blue line failed: {e}")
            return ABORT
        finally:
            handler.stop()
