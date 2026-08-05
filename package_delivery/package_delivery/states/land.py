import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavlinkDrone

from package_delivery.constants import Config

class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])


    def execute(self, blackboard: Blackboard):
        if ('drone' not in blackboard) or not blackboard['drone']:
            yasmin.YASMIN_LOG_ERROR('drone not available.')
            return ABORT
        drone: MavlinkDrone = blackboard['drone']

        yasmin.YASMIN_LOG_INFO('Landing...')

        try:
            drone.land()

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Landing failed: {e}.')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED