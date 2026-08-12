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

from hang_the_hook.hookSM.constants import(
    KP_X, KI_X, KD_X,
    KP_Y, KI_Y, KD_Y,
    KP_YAW, KI_YAW, KD_YAW,
    TIMEOUT_ALIGN,
    CENTER_TOLERANCE,
    ANGULAR_TOLERANCE,
    DROP_DIST,
    FIND_HOSE,
    DESCEND,
    ALIGN,
)
from hang_the_hook.utils.blackboard_utils import blackboard_check

class FindHose(State):

    '''
    Every call of Aling must be preceded by FindHose to update hose position.\n
    FindHose pushes the values cx, cy and angle to blackboard to be retrieved by Align.\n
    If state doesn't find a hose, ascends the drone a bit for greater fov.
    '''

    def __init__(self) -> None:
        super().__init__(outcomes=[ALIGN,ABORT])

        self.drone: MavrosDrone | MavlinkDrone
        self.camera: ImageHandler
        self.counter: int

        self.hosedetector: LineDetector

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard_check(blackboard=blackboard, args=("drone", "camera")):
            return ABORT

        self.drone        = blackboard['drone']
        self.camera       = blackboard['camera']
        self.hosedetector = blackboard['hose_detect']
        self.counter      = blackboard['findhose_state_counter']

        frame = self.camera.take_photo()

        line = self.hosedetector.detect_line(frame)[2:5]

        for i in line:
            if math.isnan(i) or not line:
                # --- Line not found
                altitude = self.drone.get_altitude()
                if altitude is not None and (altitude + 0.5 < 5.5) and self.counter <= 10:
                    self.drone.move_to(z=(altitude + 0.5))
                    self.counter += 1
                    blackboard['findhose_state_counter'] = self.counter
                    return FIND_HOSE
                break
            else:
                return ALIGN
        return ABORT

class Align(State):

    '''
    With hose pos values from FindHose, Align centers drone's camera with found line's center.\n
    Shifts center the same amount of hook-camera offset.\n
    TODO: Drone's shift by hook-camera offset
    '''

    def __init__(self) -> None:
        super().__init__(outcomes=[FIND_HOSE, DESCEND, ABORT])

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
            ):
            return ABORT

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
        self.pid_cx.set_setpoint(FRAME_WIDTH // 2)
        self.pid_cy.set_setpoint(FRAME_HEIGHT // 2)
        self.pid_angle.set_setpoint(0.0)

        # --- Starting time to apply timeout to state
        timer: float = time()

        while True:
            self.drone.delay(0.05)

            frame = self.camera.take_photo(wait_for_new=True)
            _, _, hose_x, hose_y, hose_yaw, _, _ = self.hosedetector.detect_line(frame)
            error_x = abs(hose_x - FRAME_WIDTH//2)
            error_y = abs(hose_y - FRAME_HEIGHT//2)

            if max(error_x, error_y) <= CENTER_TOLERANCE:
                return DESCEND

            # --- Calculating velocity based on hose's pos
            vy = self.pid_cx.update(hose_x)
            vx = self.pid_cy.update(hose_y)
            vyaw = self.pid_angle.update(hose_yaw)

            self.drone.move_velocity(vx=vx, vy=vy, vz=0, vyaw=vyaw)

            if timer - time() >= TIMEOUT_ALIGN:
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
        if not blackboard_check(blackboard, ('drone', 'camera', 'hose_detect')):
            return ABORT

        self.drone        = blackboard['drone']
        self.camera       = blackboard['camera']
        self.hosedetector = blackboard['hose_detect']

        while True:
            # --- First, checks for hose in camera view, just for watchdog
            frame = self.camera.take_photo(wait_for_new=True)
            line = self.hosedetector.detect_line(frame)[2:5]

            for i in line:
                if math.isnan(i) or not line:
                    return FIND_HOSE

            # --- Then, checks center tolerance, if out of returns to align
            hose_x, hose_y, hose_angle = line[0], line[1], line[2]
            error_x = abs(hose_x - FRAME_WIDTH//2)
            error_y = abs(hose_y - FRAME_HEIGHT//2)
            error_angle = hose_angle
            if max(error_x, error_y) > CENTER_TOLERANCE:
                return ALIGN
            if error_angle > ANGULAR_TOLERANCE:
                return ALIGN

            # --- If already near the hose, drop the hook
            altitude = self.drone.get_altitude()
            if altitude is not None and altitude <= DROP_DIST:
                self.drone.do_servo(
                    aux_out= 0,
                    pwm_value= 0,
                )
                return SUCCEED

            # --- None of the above applies, good to descend
            self.drone.move_velocity(vz=-0.1)
