from traceback import print_exc
from datetime import datetime

from math import dist as math_dist
import cv2
import nectar
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from nectar.vision import (
    ImageHandler,
    OpenCVConfig,
    LineDetector,
    RotatedRect,
    ColorSpace,
)

from nectar.control import (
    PIDController,
    AltitudeSource,
    MavrosDrone,
    MavlinkDrone,
    DroneFactory,
    MavrosConfig,
    PoseSource,
    MoveReference,
)

from hang_the_hook.followlineSM.constants import (
    CENTER_VARIATION,
    ANGLE_KD,
    ANGLE_KI,
    ANGLE_KP,
    CX_KD,
    CX_KI,
    CX_KP,
    FRAME_WIDTH,
    FRAME_HEIGHT,
    MIN_BLUE_FRAMES,
    MIN_RED_FRAMES,
    FOWARD_SPEED_BLUE_LINE,
    HOSE_COUNTER,
    FOUND_RED,
    FOUND_BLUE
)

class SearchBlueLine(State):
    def __init__(self):
        super().__init__(outcomes=[FOUND_BLUE, FOUND_RED, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
            # Retrieve the line linedetector and image handler from the Blackboard
            linedetector: LineDetector = blackboard["line_detect"]
            hosedetector: LineDetector = blackboard["hose_detect"]
            camera: ImageHandler = blackboard["camera"]
            counterBlue = 0
            counterRed = 0
            oldcxBlue = 0
            oldcyBlue = 0
            oldcxRed = 0
            oldcyRed = 0

            if not linedetector or not camera:
                print("One or more detectors or image handler not initialized.")
                return ABORT

            # Start the image handler to process frames and detect lines
            camera.start()

            # Main loop for line following
            while True:
                frame = camera.take_photo()
                resultBlue, _, cxBlue, cyBlue, angleBlue, _, _ = linedetector.detect_line(frame, draw=True)
                resultRed, _, cxRed, cyRed, _, _, _ = hosedetector.detect_line(frame, draw=True)

                #Skips if no line detected
                if not resultBlue or cxBlue is None or cyBlue is None:
                    counterBlue = 0
                    continue
                
                if not resultRed or cxRed is None or cyRed is None:
                    counterRed = 0

                distanceBlue = math_dist((cxBlue,cyBlue),(oldcxBlue,oldcyBlue))
                distanceRed = math_dist((cxRed,cyRed),(oldcxRed,oldcyRed))

                if (distanceBlue < CENTER_VARIATION):
                    counterBlue += 1
                    
                if (distanceRed < CENTER_VARIATION):
                    counterRed += 1

                if counterRed == MIN_RED_FRAMES:
                    counterRed = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"../images/{now}.png", resultRed)
                    return FOUND_RED
                
                if counterBlue == MIN_BLUE_FRAMES:
                    counterBlue = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"../images/{now}.png", resultBlue)
                    blackboard["angle_blue"] = angleBlue
                    blackboard["center_y_blue"] = cyBlue     #X axis in image is Y axis in drone frame
                    break
                
                oldcxBlue = cxBlue
                oldcyBlue = cyBlue
                oldcxRed = cxRed
                oldcyRed = cyRed

            return FOUND_BLUE
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
            pid_cy: PIDController = blackboard["pid_cy"]
            pid_angle: PIDController = blackboard["pid_angle"]
            drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
            angle: float = blackboard["angle_blue"]
            cY: float = blackboard["center_y_blue"]

            if not drone:
                print("Drone not initialized.")
                return ABORT
            
            if not pid_cy or not pid_angle:
                print("PID controllers not initialized.")
                return ABORT
            
            if cY is None or cY is None or angle is None:
                print("Blue line parameters not available.")
                return ABORT
            
            dy = cY - (FRAME_WIDTH / 2)
            dyaw = angle
            
            pid_cy.set_setpoint(cY)

            while (True):
                vy = pid_cy.update(-dy)
                vyaw = pid_angle.update(-dyaw)
                drone.move_velocity(vx=FOWARD_SPEED_BLUE_LINE, vy=vy, vz=0.0, vyaw=vyaw, reference=MoveReference.BODY)
                if(vy == 0.0 and vyaw == 0.0):
                    break
                
            return SUCCEED

        except Exception as e:
            print(f"Follow blue line failed: {e}")
            try:
                drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, reference=MoveReference.BODY)
            except Exception:
                pass
            return ABORT
