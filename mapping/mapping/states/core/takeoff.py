import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone

from config import Config

from nectar.control import MavrosDrone, MavlinkDrone, MoveReference

class Takeoff(State):

    def __init__(self, config : Config):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config


    def execute(self, blackboard: Blackboard):

        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')

        else:
            yasmin.YASMIN_LOG_INFO('\033[31mDrone Type Not Found (MavrosDrone / MavlinkDrone)!\033[0m')
            return ABORT


        yasmin.YASMIN_LOG_INFO(f'Taking off to altitude: {self.config.takeoff_altitude} m ... ')

        try:
            #drone.set_home()
            #drone.arm()
            drone.takeoff(self.config.takeoff_altitude)
            drone.delay(3)


        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_INFO('    \033[31mExecution interrupted by user!\033[0m')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_INFO(f'   \033[31mTAKEOFF FAILED: {error}\033[0m')
            return ABORT

        yasmin.YASMIN_LOG_INFO('    \033[32mTakeoff Successfully Completed!\033[0m')
        return SUCCEED
