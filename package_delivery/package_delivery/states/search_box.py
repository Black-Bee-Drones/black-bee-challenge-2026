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

        if self.config.sim_mode:
            target_box = self.config.sim_target_box
        else:
            target_box = self.config.target_box

        try:
            i_box : int = blackboard.get("i_box")
            drone.move_to_gps(
                latitude=target_box[i_box][0],
                longitude=target_box[i_box][1],
                altitude=self.config.safe_altitude,
            )

            i_box += 1
            blackboard["i_box"] = i_box

            yasmin.YASMIN_LOG_INFO(f'Completed successfully. Box {i_box}/{len(target_box)}.')
            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Searching box failed: {e}")
            return ABORT
