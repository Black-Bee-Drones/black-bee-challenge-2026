from datetime import datetime
import os

from math import dist as math_dist, isnan as math_isnan
import cv2
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
    FOUND_BLUE,
    SEARCH
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

            if not linedetector or not camera or not hosedetector:
                print("One or more detectors or image handler not initialized.")
                return ABORT

            # Start the image handler to process frames and detect lines
            camera.open()

            # Main loop for line following
            while True:
                frame = camera.take_photo()
                resultBlue, _, cxBlue, cyBlue, angleBlue, _, _ = linedetector.detect_line(frame, draw=True)
                resultRed, _, cxRed, cyRed, _, _, _ = hosedetector.detect_line(frame, draw=True)

                #Skips if no line detected
                if cxBlue is None or math_isnan(cxBlue) or cyBlue is None or math_isnan(cyBlue):
                    counterBlue = 0
                else:
                    distanceBlue = math_dist((cxBlue, cyBlue), (oldcxBlue, oldcyBlue))
                    if distanceBlue < CENTER_VARIATION:
                        counterBlue += 1
                    else:
                        counterBlue=0
                    oldcxBlue = cxBlue
                    oldcyBlue = cyBlue

                if cxRed is None or math_isnan(cxRed) or cyRed is None or math_isnan(cyRed):
                    counterRed = 0
                else:
                    distanceRed = math_dist((cxRed, cyRed), (oldcxRed, oldcyRed))
                    if distanceRed < CENTER_VARIATION:
                        counterRed += 1
                    else:
                        counterRed = 0
                    oldcxRed = cxRed
                    oldcyRed = cyRed

                if counterRed >= MIN_RED_FRAMES:
                    counterRed = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"../images/{now}.png", resultRed)
                    return FOUND_RED

                if counterBlue >= MIN_BLUE_FRAMES:
                    counterBlue = 0
                    now = datetime.now().strftime("%Y%m%d_%H%M%S")
                    cv2.imwrite(f"../images/{now}.png", resultBlue)
                    blackboard["angle_blue"] = angleBlue
                    blackboard["center_x_blue"] = cxBlue     # Image X → drone lateral (Y) axis
                    break

            return FOUND_BLUE
        except Exception as e:
            print(f"Blue line searching failed: {e}")
            return ABORT
        finally:
            if camera:
                camera.close()

class FollowBlueLine(State):
    def __init__(self):
        super().__init__(outcomes=[SEARCH, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone | MavlinkDrone
        camera: ImageHandler
        try:
            drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
            pid_cy: PIDController = blackboard["pid_cy"]
            pid_angle: PIDController = blackboard["pid_angle"]
            linedetector: LineDetector = blackboard["line_detect"]
            camera: ImageHandler = blackboard["camera"]
            angle: float = blackboard["angle_blue"]
            cX: float = blackboard["center_x_blue"]

            if not drone:
                print("Drone not initialized.")
                return ABORT

            if not pid_cy or not pid_angle:
                print("PID controllers not initialized.")
                return ABORT

            if cX is None or angle is None:
                print("Blue line parameters not available.")
                return ABORT

            pid_cy.set_setpoint(FRAME_WIDTH / 2)
            pid_angle.set_setpoint(0.0)

            camera.open()
            while True:
                frame = camera.take_photo()
                resultBlue, _, cxBlue, cyBlue, angleBlue, _, _ = linedetector.detect_line(frame, draw=True)

                if cxBlue is not None and not math_isnan(cxBlue) and cyBlue is not None and not math_isnan(cyBlue):
                    vy = pid_cy.update(cxBlue)
                    vyaw = pid_angle.update(angleBlue)
                    log_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "utils", "errors", "error_log.csv"))
                    with open(log_file, "a") as f:
                        f.write(f"{cxBlue}, {angleBlue}, {datetime.now().strftime('%M:%S')}\n")
                    self.node.get_logger().info(f"Blue line detected: {cxBlue}, {cyBlue}, {angleBlue}")
                else:
                    self.node.get_logger().info("Blue line not detected, holding corrections")
                    # Hold last PID outputs — don't update with fake data
                    vy = 0.0
                    vyaw = 0.0

                drone.move_velocity(vx=FOWARD_SPEED_BLUE_LINE, vy=vy, vz=0.0, vyaw=vyaw, reference=MoveReference.BODY)

                if abs(vy) < 0.01 and abs(vyaw) < 0.01:
                    break

            return SEARCH

        except Exception as e:
            print(f"Follow blue line failed: {e}")
            if drone:
                try:
                    drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, reference=MoveReference.BODY)
                except Exception:
                    pass
            return ABORT
        finally:
            if camera:
                camera.close()
