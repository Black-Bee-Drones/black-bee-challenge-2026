import rclpy

import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros import set_ros_loggers


from mapping import (
    Config,
    MappingSM,
)


def main(args=None):
    """Entry point: bring up ROS2, run the mapping mission state machine to
    completion, and tear ROS2 down again.

    Args:
        args: Command-line arguments forwarded to `rclpy.init()` (standard
            ROS2 node args); None uses the process's own sys.argv.
    """

    rclpy.init(args=args)
    set_ros_loggers()

    config = Config.load()

    try:
        yasmin.YASMIN_LOG_INFO('Inicializing the State Machine...')

        mapping_sm = MappingSM(config)
        final_outcome = mapping_sm()

    except Exception as error:
        yasmin.YASMIN_LOG_ERROR(f'Mapping State Machine Failed: {error}!')

    else:
        if final_outcome == SUCCEED:
            yasmin.YASMIN_LOG_INFO(final_outcome)
        else:
            yasmin.YASMIN_LOG_ERROR(final_outcome)

    YasminNode.destroy_instance()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
