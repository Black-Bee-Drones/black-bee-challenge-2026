import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone

from mapping.config import Config

class Takeoff(State):

    def __init__(self, config : Config):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config


    def execute(self, blackboard: Blackboard):

        if self.config.drone_type != 'mavros':
            yasmin.YASMIN_LOG_INFO('\033[31mDrone Type Not Found (only MavrosDrone is supported)!\033[0m')
            return ABORT

        drone: MavrosDrone = blackboard.get('drone')


        yasmin.YASMIN_LOG_INFO(f'Taking off to altitude: {self.config.takeoff_altitude} m ... ')

        try:
            drone.set_home()
            drone.arm()
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
