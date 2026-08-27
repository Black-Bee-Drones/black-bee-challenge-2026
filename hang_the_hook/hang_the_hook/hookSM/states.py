import math
from nectar.vision import ImageHandler
from nectar.vision import LineDetector
from nectar.control import PIDController, MavrosDrone, MavlinkDrone
from nectar.control.types import MoveReference

from yasmin import State, Blackboard, YASMIN_LOG_DEBUG
from yasmin_ros.basic_outcomes import SUCCEED

from hang_the_hook.core.constants import(
    IMAGE_HEIGHT,
    IMAGE_WIDTH,
)

from hang_the_hook.hookSM.constants import *
from hang_the_hook.utils.blackboard_utils import blackboard_check

class FindHose(State):

    '''
    Seek for the hose if it's off the camera view.
    '''

    def __init__(self) -> None:
        super().__init__(outcomes=[ALIGN, RTL])

        self.drone: MavrosDrone | MavlinkDrone
        self.camera: ImageHandler
        self.searching_step: int # 0: Ascend; 1: Backward; 2: Forward; 3: Leftward; 4: Rightward; 5: Stop searching

        self.hosedetector: LineDetector

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                'drone',
                'camera',
                'hose_detect',
                )
            ): return RTL

        self.drone        = blackboard['drone']
        self.camera       = blackboard['camera']
        self.hosedetector = blackboard['hose_detect']

        self.camera.open()

        self.searching_step = 0

        while True:
            frame = self.camera.take_photo(wait_for_new=True)

            # --- Detect the line and update the validity flag.
            detection = self.hosedetector.detect_line(frame, draw=False)

            if detection is not None:
                line = detection[2:5]  # Extract (center_x, center_y, angle)
                is_valid_line = not any(math.isnan(val) for val in line)
            else:
                line = ()
                is_valid_line = False

            if is_valid_line:
                return ALIGN
            else:
                YASMIN_LOG_DEBUG('No valid line found, searching...')

            # --- If the line is invalid, increase altitude and draws a cross to obtain a wider field of view.
            match self.searching_step:
                case 0:
                    altitude = self.drone.get_altitude()
                    if altitude is not None and (altitude + ALT_INCR < MAX_ALT):
                        YASMIN_LOG_DEBUG(f'Ascending to {altitude + ALT_INCR}m')
                        self.drone.move_to(z=ALT_INCR)
                    self.searching_step += 1
                case 1:
                    YASMIN_LOG_DEBUG(f'Moving {SEARCHING_STEP_DISTANCE}m forward from origin')
                    self.drone.move_to(x=SEARCHING_STEP_DISTANCE)
                    self.searching_step += 1
                case 2:
                    YASMIN_LOG_DEBUG(f'Moving {SEARCHING_STEP_DISTANCE}m backward from origin')
                    self.drone.move_to(x=-(2*SEARCHING_STEP_DISTANCE))
                    self.searching_step += 1
                case 3:
                    YASMIN_LOG_DEBUG(f'Moving {SEARCHING_STEP_DISTANCE}m leftward from origin')
                    self.drone.move_to(x=SEARCHING_STEP_DISTANCE, y=SEARCHING_STEP_DISTANCE)
                    self.searching_step += 1
                case 4:
                    YASMIN_LOG_DEBUG(f'Moving {SEARCHING_STEP_DISTANCE}m rightward from origin')
                    self.drone.move_to(y=-(2*SEARCHING_STEP_DISTANCE))
                    self.searching_step += 1
                case 5:
                    YASMIN_LOG_DEBUG(f'Searching failed, no hose found, returning to launch')
                    break

        return RTL

class Align(State):

    '''
    Aligns the drone's camera with the center of the detected line.\n
    Applies the same shift as the hook-camera offset.
    '''

    def __init__(self) -> None:
        super().__init__(outcomes=[DESCEND, FIND_HOSE, RTL])

        self.drone: MavlinkDrone | MavrosDrone
        self.camera: ImageHandler

        self.pid_cx: PIDController
        self.pid_cy: PIDController
        self.pid_angle: PIDController
        self.hosedetector: LineDetector

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                'drone', 'camera', 'hose_detect',
                'pid_cx', 'pid_cy', 'pid_angle'
                )
            ): return RTL

        self.drone        = blackboard['drone']
        self.camera       = blackboard['camera']
        self.pid_cx       = blackboard['pid_cx']
        self.pid_cy       = blackboard['pid_cy']
        self.pid_angle    = blackboard['pid_angle']
        self.hosedetector = blackboard['hose_detect']

        # --- Count consecutive aligned detections; reaching ALIGNMENT_COUNTER confirms alignment.
        alignment_counter = 0

        # --- Count consecutive detections after alignment has been lost.
        # --- Reaching ALIGNMENT_LOSS_COUNTER stops the drone and restarts the alignment attempt.
        alignment_loss_counter = 0

        # --- Count consecutive None Line detection.
        # --- After ALIGN_MAX_HOSE_LOSSES, returns FIND_HOSE
        hose_loss_counter = 0

        # --- Count realignment attempts; reaching ALIGNMENT_MAX_RETRIES triggers an RTL.
        alignment_retry_counter = 0

        self.camera.open()

        # --- Reset the PID controllers for each new Align call.
        # --- This clears their accumulated values.
        self.pid_cx.reset()
        self.pid_cy.reset()
        self.pid_angle.reset()

        # --- Reapply the gains to prevent settings from other states from carrying over.
        self.pid_cx.tune(kp=KP_X, ki=KI_X, kd=KD_X)
        self.pid_cy.tune(kp=KP_Y, ki=KI_Y, kd=KD_Y)
        self.pid_angle.tune(kp=KP_YAW, ki=KI_YAW, kd=KD_YAW)

        # --- Set the target to the camera center plus the configured shift.
        # --- The desired yaw difference between the camera and hose is 0 degrees.
        target_x = (IMAGE_WIDTH // 2)
        target_y = (IMAGE_HEIGHT // 2) + SHIFT
        self.pid_cx.set_setpoint(target_x)
        self.pid_cy.set_setpoint(target_y)
        self.pid_angle.set_setpoint(0.0)

        YASMIN_LOG_DEBUG('PID set, values:\n')
        YASMIN_LOG_DEBUG(f'Y (drives vx - Pitch): KP = {KP_Y}, KD = {KD_Y}, KI = {KI_Y}')
        YASMIN_LOG_DEBUG(f'X (drives vy - Roll): KP = {KP_X}, KD = {KD_X}, KI = {KI_X}')
        YASMIN_LOG_DEBUG(f'YAW: KP = {KP_YAW}, KD = {KD_YAW}, KI = {KI_YAW}')
        YASMIN_LOG_DEBUG('Starting ALIGN iteration')

        while True:

            # --- No additional delay is needed because take_photo waits for a frame.
            frame = self.camera.take_photo(wait_for_new=True)

            # --- Detect the line and update the validity flag.
            detection = self.hosedetector.detect_line(frame, draw=False)

            if detection is not None:
                line = detection[2:5]  # Extract (center_x, center_y, angle)
                is_valid_line = not any(math.isnan(val) for val in line)
            else:
                line = ()
                is_valid_line = False

            if is_valid_line and line:
                hose_x, hose_y, hose_yaw = line
                hose_loss_counter = 0
            else:
                hose_loss_counter += 1
                YASMIN_LOG_DEBUG(f'While aligning there was no valid line found, losses: {hose_loss_counter}')

            if hose_loss_counter >= ALIGNMENT_MAX_NONE_LINE_DETECTION:
                YASMIN_LOG_DEBUG(f'Hose lost, attempts of static search: {hose_loss_counter}')
                self.drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                return FIND_HOSE

            if not (is_valid_line and line):
                continue

            error_x = abs(hose_x - target_x)
            error_y = abs(hose_y - target_y)

            # --- Check whether the drone is within the position and yaw tolerances.
            # --- If so, increment the counter so subsequent iterations confirm stability.
            if max(error_x, error_y) <= CENTER_TOLERANCE and abs(hose_yaw) < ANGULAR_TOLERANCE:
                alignment_counter += 1
                YASMIN_LOG_DEBUG(f'Seems aligned, alignment_counter = {alignment_counter}')
            elif alignment_counter > 0:
                alignment_counter = 0
                alignment_loss_counter += 1
                YASMIN_LOG_DEBUG(f'Aligment lost, retrying to align... loss_counter = {alignment_loss_counter}')

            # --- If alignment is lost too often, the PID response may be too aggressive.
            # --- Stop the drone briefly before retrying the alignment.
            if alignment_loss_counter >= ALIGNMENT_MAX_LOSS:
                self.drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0, duration=ALIGNMENT_RETRY_PAUSE)
                self.pid_cx.reset()
                self.pid_cy.reset()
                self.pid_angle.reset()
                alignment_loss_counter = 0
                alignment_retry_counter += 1
                YASMIN_LOG_DEBUG(f'Lost too many alignments, PID may be too aggressive, pausing drone to retry. Current attempt: {alignment_retry_counter}')
                continue

            # --- If realignment does not succeed within the retry limit, return to launch.
            if alignment_retry_counter >= ALIGNMENT_MAX_RETRIES:
                YASMIN_LOG_DEBUG('Too much attempts of realign, returning to launch...')
                return RTL

            # --- The drone appears stable, so it is safe to descend.
            if alignment_counter >= ALIGNMENT_MIN_FRAMES:
                YASMIN_LOG_DEBUG('Drone aligned')
                self.drone.move_velocity(vx=0.0, vy=0.0, vz=0.0, vyaw=0.0, duration=0.5)
                return DESCEND

            # --- After the checks above, run the alignment PID controllers.
            # --- Calculate velocity from the hose position.
            # --- Camera X controls drone Y (roll).
            # --- Camera Y controls drone X (pitch).
            vy = self.pid_cx.update(hose_x)
            vx = self.pid_cy.update(hose_y)
            vyaw = self.pid_angle.update(hose_yaw)

            self.drone.move_velocity(vx=vx, vy=vy, vz=0, vyaw=vyaw)

class Descend(State):

    '''
    Descends in a controlled manner while checking alignment with the hose.
    '''

    def __init__(self) -> None:
        super().__init__(outcomes=[SUCCEED, ALIGN, FIND_HOSE, RTL])

        self.drone: MavlinkDrone | MavrosDrone
        self.camera: ImageHandler

        self.hosedetector: LineDetector

    def execute(self, blackboard: Blackboard) -> str:
        if not blackboard_check(
            blackboard=blackboard,
            args=(
                'drone',
                'camera',
                'hose_detect'
                )
            ): return RTL

        self.drone        = blackboard['drone']
        self.camera       = blackboard['camera']
        self.hosedetector = blackboard['hose_detect']

        altitude_reference_loss_counter = 0
        hose_loss_counter = 0

        self.camera.open()
        target_x = (IMAGE_WIDTH // 2)
        target_y = (IMAGE_HEIGHT // 2) + SHIFT

        while True:
            # --- No additional delay is needed because take_photo waits for a frame.
            frame = self.camera.take_photo(wait_for_new=True)

            # --- Detect the line and update the validity flag.
            detection = self.hosedetector.detect_line(frame, draw=False)

            if detection is not None:
                line = detection[2:5]  # Extract (center_x, center_y, angle)
                is_valid_line = not any(math.isnan(val) for val in line)
            else:
                line = ()
                is_valid_line = False

            if is_valid_line and line:
                hose_x, hose_y, hose_yaw = line
                hose_loss_counter = 0
            else:
                hose_loss_counter += 1
                YASMIN_LOG_DEBUG(f'While descending there was no valid line found, losses: {hose_loss_counter}')

            if hose_loss_counter >= DESCEND_MAX_NONE_LINE_DETECTION:
                YASMIN_LOG_DEBUG(f'Hose lost, attempts of static search: {hose_loss_counter}, switching to dynamic search')
                self.drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                return FIND_HOSE

            if not (is_valid_line and line):
                continue

            error_x = abs(hose_x - target_x)
            error_y = abs(hose_y - target_y)

            if max(error_x, error_y) > CENTER_TOLERANCE or abs(hose_yaw) > ANGULAR_TOLERANCE:
                self.drone.move_velocity(vx=0, vy=0, vz=0, vyaw=0)
                YASMIN_LOG_DEBUG('Hose is out of alignment, switching to ALIGN state')
                return ALIGN

            altitude = self.drone.get_altitude()

            # --- If the drone is close enough to the hose, release the hook.
            if altitude is not None:
                if altitude <= DROP_DIST:
                    self.drone.do_servo(
                        aux_out= AUX_OUT,
                        pwm_value= PWM_VALUE,
                    )
                    return SUCCEED
                else:
                    # --- Otherwise, continue descending.
                    self.drone.move_velocity(vz=-DESCEND_STEP)
