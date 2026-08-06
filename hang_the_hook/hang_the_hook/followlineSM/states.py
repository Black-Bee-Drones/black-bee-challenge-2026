from traceback import print_exc

from math import dist as math_dist
from hang_the_hook.core.constants import(
    FRAME_WIDTH,
    FRAME_HEIGHT,
    IMAGE_SOURCE,
)

from hang_the_hook.followlineSM.constants import (
    CENTER_VARIATION,
    HOSE_AREA,
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
)

from hang_the_hook.utils.line_follow import segue_linha
from cv2 import imwrite as cv2_imwrite

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from datetime import datetime


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

                if counterblue == 5:
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

    def execute(self, blackboard: Blackboard):
        try:
            pid_cx: PIDController = blackboard["pid_cx"]
            pid_angle: PIDController = blackboard["pid_angle"]
            camera: ImageHandler = blackboard["camera"]
            drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
            linedetector: LineDetector = blackboard["line_detect"]
            hosedetector: LineDetector = blackboard["hose_detect"]

            if not camera:
                print("Image handler not initialized.")
                return ABORT

            camera.start()

            while True:
                frame = camera.take_photo()
                resultb, _, cx, cy, angle, w, h = linedetector.detect_line(frame, draw=True)

                if cx is None:
                    # perdeu a linha -> volta pra SEARCH_BLUE_LINE
                    return ABORT

                #TODO: Logic PID
                '''
                vx = self.pid_cx.update(cx)
                vyaw = self.pid_angle.update(angle)

                ^^^^^^^^^^^^ essas funções já fizeram a conversão pro ideal (meio da imagem e 0 graus, que é o setpoint), falta transformar essas variáveis de velocidade em movimentação no drone.

                drone.move_velocity(vx=vx, vy=0.0, vz=0.0, vyaw=vyaw, reference=MoveReference.BODY)
                ^^^^^algo assim, eu acho [a gente tem o retorno de cy tbm, talvez dê pra fazer algo com ele]
                '''

                _, _, hose_cx, _, _, _, _ = hosedetector.detect_line(frame, draw=True)

                if hose_cx is not None:
                        HOSE_COUNTER += 1
                    else:
                        HOSE_COUNTER = 0
                else:
                    HOSE_COUNTER = 0

                if HOSE_COUNTER >= FRAMES_TO_CONFIRM_HOSE:
                    print("Hose detected! Stopping line following.")
                    return SUCCEED

        except Exception as e:
            print(f"Follow blue line failed: {e}")
            return ABORT
        finally:
            camera.stop()
