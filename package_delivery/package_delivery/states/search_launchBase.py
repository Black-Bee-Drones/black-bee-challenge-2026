import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from nectar.control import MavrosDrone, MoveReference
from package_delivery.constants import Config


class SearchLaunchBase(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config
        self._i_base = 0

    def execute(self, blackboard: Blackboard):
        drone: MavrosDrone = blackboard.get('drone')

        target = self.config.launch_bases[self._i_base]
        self._i_base += 1

        altitude = self.config.safe_altitude

        yasmin.YASMIN_LOG_INFO(f'Going to Base: {target}...')
        drone.move_to(
            x=target.get('x'),
            y=target.get('y'),
            z=altitude,
            reference=MoveReference.TAKEOFF,
        )

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED
