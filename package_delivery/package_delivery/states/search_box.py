import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from nectar.control import MavrosDrone, MavlinkDrone, MoveReference
from package_delivery.constants import Config


class SearchBox(State):
    def __init__(self, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
        
        else:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT 

        # yasmin.YASMIN_LOG_INFO(f'Going to Box: {target}...')
        drone.move_to_gps(
            latitude=self.config.target_box[0],
            longitude=self.config.target_box[1],
            altitude=self.config.safe_altitude,
        )
        
        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED