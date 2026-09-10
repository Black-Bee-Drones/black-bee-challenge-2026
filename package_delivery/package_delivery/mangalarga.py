import rclpy
import nectar
import yasmin

from yasmin import StateMachine
from yasmin_ros import set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from package_delivery.states.core import (
    Initialize,
    Takeoff,
    Land,
    Rtl,
)

from package_delivery.states import (
    SearchBox,
    Approach,
    Gripper,
    Wait,
)


class PackageDelivery(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={SUCCEED: "TAKEOFF", ABORT: ABORT}
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={SUCCEED: "SEARCH_BOX", ABORT: "RTL", "END": SUCCEED}

        )

        self.add_state(
            "SEARCH_BOX",
            SearchBox(),
            transitions={SUCCEED: "APPROACH", ABORT: "LAND"}
        )

        self.add_state(
            "APPROACH",
            Approach(),
            transitions={SUCCEED: "DROP_PKG", ABORT: "RTL"}
        )

        self.add_state(
            "DROP_PKG",
            Gripper(target_has_pkg=False),
            transitions={SUCCEED: "RTL", ABORT: "RTL"}
        )

        self.add_state(
            "RTL",
            Rtl(),
            transitions={SUCCEED: "WAIT", ABORT: "LAND"}
        )

        self.add_state(
            "WAIT",
            Wait(),
            transitions={SUCCEED: "TAKEOFF", ABORT: ABORT}
        )

        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT}
        )

        self.set_start_state("INITIALIZE")


def main():
    rclpy.init()

    nectar.use_executor(YasminNode.get_instance()._executor)

    set_ros_loggers()

    package_delivery_sm = PackageDelivery()

    try:
        final_outcome = package_delivery_sm()
        yasmin.YASMIN_LOG_INFO(final_outcome)
    except KeyboardInterrupt:
        if package_delivery_sm.is_running():
            package_delivery_sm.cancel_state()
    finally:
        nectar.shutdown()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()