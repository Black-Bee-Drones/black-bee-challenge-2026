import rclpy
import nectar
import yasmin

from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin import State, Blackboard
from package_delivery.constants import Config

class Wait(State):
    def __init__(self, config: Config = Config, ):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        while True:
            try:
                answer = input("Type 'yes' when manually fixed the package: ").strip().lower()
                if answer == "yes":
                    blackboard['has_thePkg'] = True
                    return SUCCEED
                print("Type exactally 'yes' to continue or (Ctrl + C) to ABORT.")
            
            except KeyboardInterrupt:
                yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
                return ABORT
            
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Waiting failed: {e}")
                return ABORT           

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
            # drone.move_to(z=-0.8)
            # drone.move_velocity(0.0, 0.0, 0.0)
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

class TestWait(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "WAIT",
            Wait(),
            transitions={SUCCEED: "GRIPPER", ABORT: ABORT, "END": SUCCEED}
        )

        self.add_state(
            "GRIPPER",
            Gripper(target_has_pkg=False),
            transitions={SUCCEED: SUCCEED, ABORT: "WAIT"}
        )

        self.set_start_state("WAIT")


def main():
    rclpy.init()

    nectar.use_executor(YasminNode.get_instance()._executor)

    set_ros_loggers()

    test_sm = TestWait()

    try:
        final_outcome = test_sm()
        yasmin.YASMIN_LOG_INFO(final_outcome)
    except KeyboardInterrupt:
        if test_sm.is_running():
            test_sm.cancel_state()
    finally:
        nectar.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()