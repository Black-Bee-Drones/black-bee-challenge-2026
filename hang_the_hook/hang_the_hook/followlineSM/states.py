from datetime import datetime
from time import time
import os
import math

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
    SEARCH,
    SEEK,
    SEEK_SQUARE_BASE_SIDE,
    SEEK_SQUARE_SPEED,
    SEEK_MAX_SQUARES,
    SEEK_SQUARE_GROWTH,
    MAX_LOST_FRAMES,
    SEARCH_TIMEOUT,
)

class SearchBlueLine(State):
    '''
    Searches for both blue and red lines while the drone is hovering.
    Captures frames continuously and counts consecutive stable detections
    (center position within CENTER_VARIATION pixels).
    Returns FOUND_BLUE when the blue line is confirmed (MIN_BLUE_FRAMES),
    FOUND_RED when the red hose is confirmed (MIN_RED_FRAMES),
    SEEK if no line is detected within SEARCH_TIMEOUT seconds,
    or ABORT on any failure.
    '''

    def __init__(self):
        super().__init__(outcomes=[FOUND_BLUE, FOUND_RED, SEEK, ABORT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
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

            last_detection_time = time()

            while True:
                if (time() - last_detection_time) >= SEARCH_TIMEOUT:
                    self.node.get_logger().warn(
                        f"SearchBlueLine: no detection for {SEARCH_TIMEOUT}s — transitioning to SEEK"
                    )
                    return SEEK

                frame = camera.take_photo()
                resultBlue, _, cxBlue, cyBlue, angleBlue, _, _ = linedetector.detect_line(frame, draw=True)
                resultRed, _, cxRed, cyRed, _, _, _ = hosedetector.detect_line(frame, draw=True)

                any_detection = False

                if cxBlue is None or math_isnan(cxBlue) or cyBlue is None or math_isnan(cyBlue):
                    counterBlue = 0
                else:
                    any_detection = True
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
                    any_detection = True
                    distanceRed = math_dist((cxRed, cyRed), (oldcxRed, oldcyRed))
                    if distanceRed < CENTER_VARIATION:
                        counterRed += 1
                    else:
                        counterRed = 0
                    oldcxRed = cxRed
                    oldcyRed = cyRed

                if any_detection:
                    last_detection_time = time()

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
                    blackboard["center_x_blue"] = cxBlue
                    break

            return FOUND_BLUE
        except Exception as e:
            print(f"Blue line searching failed: {e}")
            return ABORT

class SeekLine(State):
    '''
    Recovery state activated when the drone loses sight of the blue line.
    Flies a growing square pattern centered on the point where the line
    was lost. Each iteration the square side grows by SEEK_SQUARE_GROWTH.
    Up to SEEK_MAX_SQUARES attempts are made (default 3).
    During every straight leg the camera keeps scanning for both blue
    and red lines. Returns FOUND_BLUE / FOUND_RED on detection, or
    ABORT after all squares are exhausted without finding any line.
    '''

    def __init__(self):
        super().__init__(outcomes=[FOUND_BLUE, FOUND_RED, ABORT])
        self.node = YasminNode.get_instance()

    def _check_lines(self, frame, linedetector, hosedetector, counters):
        blue_count, red_count = counters

        _, _, cxBlue, _, angleBlue, _, _ = linedetector.detect_line(frame, draw=False)
        _, _, cxRed, _, _, _, _ = hosedetector.detect_line(frame, draw=False)

        if cxBlue is not None and not math_isnan(cxBlue):
            blue_count += 1
            if blue_count >= MIN_BLUE_FRAMES:
                return (FOUND_BLUE, angleBlue, cxBlue), (blue_count, red_count)
        else:
            blue_count = 0

        if cxRed is not None and not math_isnan(cxRed):
            red_count += 1
            if red_count >= MIN_RED_FRAMES:
                return (FOUND_RED, None, None), (blue_count, red_count)
        else:
            red_count = 0

        return None, (blue_count, red_count)

    def _fly_leg(self, drone, camera, linedetector, hosedetector,
                 leg_duration, counters):
        leg_start = time()
        while (time() - leg_start) < leg_duration:
            frame = camera.take_photo()
            if frame is not None:
                result, counters = self._check_lines(
                    frame, linedetector, hosedetector, counters
                )
                if result is not None:
                    return result, counters
            drone.move_velocity(
                vx=SEEK_SQUARE_SPEED, vy=0.0, vz=0.0, vyaw=0.0,
                reference=MoveReference.BODY,
            )
        drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0)
        return None, counters

    def execute(self, blackboard: Blackboard):
        try:
            drone: MavrosDrone | MavlinkDrone = blackboard["drone"]
            camera: ImageHandler = blackboard["camera"]
            linedetector: LineDetector = blackboard["line_detect"]
            hosedetector: LineDetector = blackboard["hose_detect"]

            if not drone or not camera or not linedetector or not hosedetector:
                self.node.get_logger().error("SeekLine: missing blackboard entries")
                return ABORT

            drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0)
            self.node.get_logger().warn("SeekLine: line lost — starting square search")

            counters = (0, 0)

            for sq in range(SEEK_MAX_SQUARES):
                side = SEEK_SQUARE_BASE_SIDE * (SEEK_SQUARE_GROWTH ** sq)
                leg_duration = side / SEEK_SQUARE_SPEED
                self.node.get_logger().info(
                    f"SeekLine: square {sq + 1}/{SEEK_MAX_SQUARES}  "
                    f"side={side:.1f}m  leg_time={leg_duration:.1f}s"
                )

                for leg in range(4):
                    result, counters = self._fly_leg(
                        drone, camera, linedetector, hosedetector,
                        leg_duration, counters,
                    )
                    if result is not None:
                        outcome, angleBlue, cxBlue = result
                        drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0)
                        if outcome == FOUND_BLUE:
                            blackboard["angle_blue"] = angleBlue
                            blackboard["center_x_blue"] = cxBlue
                            self.node.get_logger().info("SeekLine: blue line recovered")
                        else:
                            self.node.get_logger().info("SeekLine: red line found")
                        return outcome

                    drone.move_to(yaw=-90, reference=MoveReference.BODY)

            drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0)
            self.node.get_logger().error(
                f"SeekLine: {SEEK_MAX_SQUARES} squares completed — aborting"
            )
            return ABORT

        except Exception as e:
            self.node.get_logger().error(f"SeekLine failed: {e}")
            return ABORT

class FollowBlueLine(State):
    '''
    Follows the detected blue line using PID controllers for lateral
    (vy) and yaw (vyaw) corrections while moving forward at constant
    speed. Continuously checks each frame for the blue line. If the
    line is not detected for MAX_LOST_FRAMES consecutive frames, stops
    the drone and transitions to SEEK. When corrections converge
    (vy ≈ 0, vyaw ≈ 0), transitions back to SEARCH to re-confirm
    the line position.
    '''

    def __init__(self):
        super().__init__(outcomes=[SEARCH, SEEK, ABORT])
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
            pid_cy.tune(CX_KP, CX_KI, CX_KD)
            pid_angle.tune(ANGLE_KP, ANGLE_KI, ANGLE_KD)

            lost_frames = 0
            log_file = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "utils", "errors", "error_log.csv"))
            while True:
                frame = camera.take_photo()
                resultBlue, _, cxBlue, cyBlue, angleBlue, _, _ = linedetector.detect_line(frame, draw=True)

                if cxBlue is not None and not math_isnan(cxBlue) and cyBlue is not None and not math_isnan(cyBlue):
                    lost_frames = 0
                    vy = pid_cy.update(cxBlue)
                    vyaw = pid_angle.update(angleBlue)
                    cx_error = cxBlue - FRAME_WIDTH / 2
                    with open(log_file, "a") as f:
                        f.write(f"{cx_error}, {angleBlue}, {datetime.now().strftime('%M:%S')}\n")
                    self.node.get_logger().info(f"Blue line detected: {cxBlue}, {cyBlue}, {angleBlue}")
                else:
                    lost_frames += 1
                    self.node.get_logger().info(
                        f"Blue line not detected ({lost_frames}/{MAX_LOST_FRAMES})"
                    )
                    if lost_frames >= MAX_LOST_FRAMES:
                        drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, reference=MoveReference.BODY)
                        self.node.get_logger().warn("Line lost — transitioning to SEEK")
                        return SEEK
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
