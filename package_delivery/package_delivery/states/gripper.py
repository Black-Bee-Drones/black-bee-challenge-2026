import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from package_delivery.constants import Config


def do_gripper(drone, config: Config, closed: bool) -> bool:
    pwm = config.servo_closed_pwm if closed else config.servo_open_pwm
    retries = max(1, int(config.servo_retries))
    for attempt in range(1, retries + 1):
        yasmin.YASMIN_LOG_INFO(f"Set {pwm} pwm (attempt {attempt}/{retries})")
        if drone.do_servo(aux_out=config.servo_channel, pwm_value=pwm):
            drone.delay(config.servo_action_delay)
            return True
        yasmin.YASMIN_LOG_WARN(f"do_servo failed (attempt {attempt}/{retries})")
        if attempt < retries:
            drone.delay(config.servo_retry_delay)
    yasmin.YASMIN_LOG_ERROR("Failed to do servo.")
    return False


class Gripper(State):
    def __init__(self, target_has_pkg: bool, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config
        self.target_has_pkg = target_has_pkg

    def execute(self, blackboard: Blackboard):
        drone = blackboard.get("drone")
        if drone is None:
            yasmin.YASMIN_LOG_ERROR("Drone missing.")
            return ABORT

        if "has_thePkg" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Flag - has the pkg - not found.")
            return ABORT

        try:
            drone.move_to(z=drone.get_altitude() - 0.5)
            drone.move_velocity(0.0, 0.0, 0.0)
            if not do_gripper(drone, self.config, self.target_has_pkg):
                return ABORT
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN("Execution interrupted by user.")
            return ABORT
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Gripper failed: {e}")
            return ABORT

        blackboard["has_thePkg"] = self.target_has_pkg
        yasmin.YASMIN_LOG_INFO("Completed successfully.")
        return SUCCEED
