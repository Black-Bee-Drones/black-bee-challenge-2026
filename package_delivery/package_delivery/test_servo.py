"""Bench-test the mission Gripper state (open then close)."""

import logging
import sys
from pathlib import Path

import nectar
from nectar.control import DroneFactory, MavlinkConfig, PoseSource
from yasmin import Blackboard
from yasmin_ros.basic_outcomes import SUCCEED

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from package_delivery.constants import Config
from package_delivery.states.gripper import Gripper

log = logging.getLogger("test_servo")


def main() -> None:
    logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
    config = Config()

    nectar.init()
    drone = None
    try:
        drone = DroneFactory.create(
            config.drone_type,
            MavlinkConfig(
                pose_source=PoseSource.GPS,
                connection_string=config.connection_string,
                expect_lidar=False,
                sensor_timeout=2.0,
            ),
        )
        blackboard = Blackboard()
        blackboard["drone"] = drone
        blackboard["has_thePkg"] = True

        for closed, name in ((False, "open"), (True, "close")):
            outcome = Gripper(target_has_pkg=closed, config=config).execute(blackboard)
            log.info("Gripper %s: %s", name, outcome)
            if outcome != SUCCEED:
                return
    except KeyboardInterrupt:
        log.warning("Interrupted")
    finally:
        if drone is not None:
            drone.cleanup()
        nectar.shutdown()


if __name__ == "__main__":
    main()
