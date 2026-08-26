import rclpy
import yasmin

from yasmin import Blackboard
from yasmin_ros.yasmin_node import YasminNode

from nectar.control import DroneFactory, SITL_GAZEBO_CONFIG, MavlinkConfig, PoseSource
from package_delivery.constants import Config
from package_delivery.states.gripper import Gripper


def main():
    rclpy.init()

    node = YasminNode.get_instance()

    config = Config()
    drone_config = MavlinkConfig(
        pose_source=PoseSource.VISION,
        connection_string=config.connection_string
    )

    drone = DroneFactory.create(
        config.drone_type,
        drone_config
    )

    blackboard = Blackboard()

    # Valores necessários pelo Gripper
    blackboard["drone"] = drone
    blackboard["has_thePkg"] = True

    # False -> servo_open_pwm
    # True  -> servo_closed_pwm
    gripper = Gripper(
        target_has_pkg=True,
        config=config
    )

    outcome = gripper.execute(blackboard)

    yasmin.YASMIN_LOG_INFO(f"Gripper outcome: {outcome}")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
