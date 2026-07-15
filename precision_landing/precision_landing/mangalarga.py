# Main state machine that will control the drone
import rclpy
import yasmin
from yasmin_ros import set_ros_loggers
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT, ABORT

from precision_landing.states import (
    Initialize,
    Takeoff,
    Find,
    Precision_landing,
    Land,
)

class PL(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED:"TAKEOFF", ABORT:ABORT},
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED:"FIND", ABORT:"LAND"},
        )

        self.add_state(
            "FIND",
            Find(),
            transitions={SUCCEED:"PRECISION_LANDING", ABORT:"LAND"},
        )

        self.add_state(
            "PRECISION_LANDING",
            Precision_landing(),
            transitions={SUCCEED:SUCCEED, ABORT:"LAND"},
        )

        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED:SUCCEED, ABORT:ABORT},
        )

        self.set_start_state("INITIALIZE")

def main():
    rclpy.init()

    set_ros_loggers()
    pl_sm = PL()

    try:
        final_outcome = pl_sm()
        yasmin.YASMIN_LOG_INFO(final_outcome)
    except KeyboardInterrupt:
        if pl_sm.is_running():
            pl_sm.cancel_state()
    
    if rclpy.ok():
        rclpy.shutdown()

if (__name__ == "__main__"):
    main()