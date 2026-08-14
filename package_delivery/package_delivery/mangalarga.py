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
    Delivery,
    SearchLaunchBase,
    PrecisionLand,
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
            transitions={SUCCEED: "SEARCH_BOX", ABORT: "LAND"}

        )

        self.add_state(
            "SEARCH_BOX",
            SearchBox(),
            transitions={SUCCEED: "APPROACH", ABORT: "LAND"}
        )

        self.add_state(
            "APPROACH",
            Approach(),
            transitions={SUCCEED: "DELIVERY", ABORT: "LAND"}
        )

        self.add_state(
            "DELIVERY",
            Delivery(),
            transitions={SUCCEED: "SEARCH_LAUNCH_BASE", ABORT: "LAND"}
        )

        self.add_state(
            "SEARCH_LAUNCH_BASE",
            SearchLaunchBase(),
            transitions={SUCCEED: "PRECISION_LAND", ABORT: "LAND"}
        )

        self.add_state(
            "PRECISION_LAND",
            PrecisionLand(),
            transitions={SUCCEED: "LAND", ABORT: "LAND"}
        )

        self.add_state(
            "LAND",
            Land(),
            transitions={SUCCEED: "LAND", ABORT: ABORT}
        )

        # This state must have different outcomes.
        self.add_state(
            "WAIT",
            Wait(),
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

    if rclpy.ok():
        rclpy.shutdown()

if __name__ == "__main__":
    main()