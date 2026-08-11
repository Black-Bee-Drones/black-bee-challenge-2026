import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone, MavlinkDrone

from package_delivery.constants import Config

class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])


    def execute(self, blackboard: Blackboard):
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
        
        else:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT

        yasmin.YASMIN_LOG_INFO('Landing...')

        try:
            drone.land()

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Landing failed: {e}.')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED