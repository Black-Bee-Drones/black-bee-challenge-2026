import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from nectar.control import MavrosDrone

from package_delivery.constants import Config


class Takeoff(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        yasmin.YASMIN_LOG_INFO(
            f'Taking off to altitude: {self.config.takeoff_altitude} m...')

        try:
            drone.takeoff(self.config.takeoff_altitude)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Taking off failed: {e}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED