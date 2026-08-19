from time import time

import math
from nectar.vision import ImageHandler
from nectar.vision import LineDetector
from nectar.control import PIDController, MavrosDrone, MavlinkDrone

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from hang_the_hook.core.constants import(
    FRAME_HEIGHT,
    FRAME_WIDTH,
)

from hang_the_hook.hookSM.constants import *
from hang_the_hook.utils.blackboard_utils import blackboard_check

class FindHose(State):

    '''
    Each call to `Align` must be preceded by FindHose to update the hose's position.\n
    FindHose ensures the hose is within the camera's field of view.\n
    If the state fails to find a hose, the drone ascends slightly to obtain a wider field of view.
    '''

    def __init__(self) -> None:
        super().__init__(outcomes=[FIND_HOSE, ALIGN, ABORT])

        self.drone: MavrosDrone | MavlinkDrone
        self.camera: ImageHandler
        self.counter: int

        self.hosedetector: LineDetector

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                "drone",
                "camera",
                'hose_detect',
                )
            ): return ABORT

        self.drone        = blackboard['drone']
        self.camera       = blackboard['camera']
        self.hosedetector = blackboard['hose_detect']

        frame = self.camera.take_photo()

        # --- Runs detect line and update is_valid flag
        detected_line = self.hosedetector.detect_line(frame)[2:5]
        line = detected_line[2:5] if detected_line and len(detected_line) >= 5 else []
        is_valid_line = len(line) == 3 and not any(math.isnan(val) for val in line)

        if is_valid_line:
            return ALIGN

        # --- If line is not valid, increase altitude for greater fov
        altitude = self.drone.get_altitude()
        if altitude is not None and (altitude + ALT_INCR < MAX_ALT):
            self.drone.move_to(z=(altitude + ALT_INCR))
            blackboard['findhose_state_counter'] = self.counter + 1
            return FIND_HOSE

        return ABORT

class Align(State):

    '''
    Align centers drone's camera with found line's center.\n
    Shifts center by the same amount of hook-camera offset.\n
    '''

    def __init__(self) -> None:
        super().__init__(outcomes=[DESCEND, ABORT])

        self.drone: MavlinkDrone | MavrosDrone
        self.camera: ImageHandler

        self.pid_cx: PIDController
        self.pid_cy: PIDController
        self.pid_angle: PIDController
        self.hosedetector: LineDetector

        self.timeout: float = TIMEOUT_ALIGN

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                'drone', 'camera', 'hose',
                'pid_cx', 'pid_cy', 'pid_angle',
                )
            ): return ABORT

        self.drone        = blackboard["drone"]
        self.camera       = blackboard["camera"]
        self.pid_cx       = blackboard["pid_cx"]
        self.pid_cy       = blackboard["pid_cy"]
        self.pid_angle    = blackboard["pid_angle"]
        self.hosedetector = blackboard["hose_detect"]

        # --- Reseting PID for every new Align call
        # --- Grants PID values to be 0
        self.pid_cx.reset()
        self.pid_cy.reset()
        self.pid_angle.reset()

        # --- Applying gains reset to prevent misscall from other states
        self.pid_cx.tune(kp=KP_X, ki=KI_X, kd=KD_X)
        self.pid_cy.tune(kp=KP_Y, ki=KI_Y, kd=KD_Y)
        self.pid_angle.tune(kp=KP_YAW, ki=KI_YAW, kd=KD_YAW)

        # --- The desired alignment is between hose's and camera's center, plus shift.
        # --- And also 0 degrees of yaw between camera-hose
        target_x = (FRAME_WIDTH // 2) + SHIFT
        target_y = (FRAME_HEIGHT // 2) + SHIFT
        self.pid_cx.set_setpoint(target_x)
        self.pid_cy.set_setpoint(target_y)
        self.pid_angle.set_setpoint(0.0)

        # --- Starting time to apply timeout to state
        timer: float = time()

        while True:

            # --- Delay isn't necessery due to take_photo
            frame = self.camera.take_photo(wait_for_new=True)
            detection = self.hosedetector.detect_line(frame, draw=False)
            if detection is None or None in detection:
                self.drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                return FIND_HOSE
            else:
                _, _, hose_x, hose_y, hose_yaw, _, _ = detection

                error_x = abs(hose_x - target_x)
                error_y = abs(hose_y - target_y)

                if max(error_x, error_y) <= CENTER_TOLERANCE and abs(hose_yaw) < ANGULAR_TOLERANCE:
                    self.drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                    return DESCEND

                # --- Calculating velocity based on hose's pos
                # --- Assumes camera X drives Drone Y (roll)
                # --- And camera Y drives Drone X (pitch)
                vy = self.pid_cx.update(hose_x)
                vx = self.pid_cy.update(hose_y)
                vyaw = self.pid_angle.update(hose_yaw)

                self.drone.move_velocity(vx=vx, vy=vy, vz=0, vyaw=vyaw)

            if time() - timer >= TIMEOUT_ALIGN:
                return DESCEND

class Descend(State):

    '''
    Drone descends in a controlled manner, checking its alignment with the hose
    '''

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, ALIGN, FIND_HOSE])

        self.drone: MavlinkDrone | MavrosDrone
        self.camera: ImageHandler

        self.hosedetector: LineDetector

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                'drone',
                'camera',
                'hose_detect',
                )
            ): return ABORT

        self.drone        = blackboard['drone']
        self.camera       = blackboard['camera']
        self.hosedetector = blackboard['hose_detect']

        target_x = (FRAME_WIDTH // 2) + SHIFT
        target_y = (FRAME_HEIGHT // 2) + SHIFT

        while True:

            frame = self.camera.take_photo(wait_for_new=True)
            detection = self.hosedetector.detect_line(frame, draw=False)
            if detection is None or None in detection:
                self.drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                return FIND_HOSE
            else:
                _, _, hose_x, hose_y, hose_yaw, _, _ = detection

                error_x = abs(hose_x - target_x)
                error_y = abs(hose_y - target_y)

                if max(error_x, error_y) > CENTER_TOLERANCE or abs(hose_yaw) > ANGULAR_TOLERANCE:
                    self.drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                    return ALIGN

            # --- If already near the hose, drop the hook
            altitude = self.drone.get_altitude()
            if altitude is not None and altitude <= DROP_DIST:
                self.drone.do_servo(
                    aux_out= AUX_OUT,
                    pwm_value= PWM_VALUE,
                )
                return SUCCEED

            # --- None of the above applies, good to descend
            self.drone.move_velocity(vz=-0.1)
