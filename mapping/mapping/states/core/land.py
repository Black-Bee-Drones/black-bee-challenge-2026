import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone

from mapping.config import Config, LandingMode


class Land(State):

    def __init__(self, config : Config):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config


    def execute(self, blackboard: Blackboard):

        if self.config.drone_type != 'mavros':
            yasmin.YASMIN_LOG_INFO('\033[31mDrone Type Not Found (only MavrosDrone is supported)!\033[0m')
            return ABORT

        drone: MavrosDrone = blackboard.get('drone')

        yasmin.YASMIN_LOG_INFO('Landing...')

        try:
            if self.config.landing_mode == LandingMode.RTL:
                drone.rtl()
            else:
                drone.land()

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_INFO('    \033[31mExecution interrupted by user!\033[0m')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_INFO(f'   \033[31mLAND FAILED: {error}\033[0m')
            return ABORT

        yasmin.YASMIN_LOG_INFO('    \033[32mLAND Successfully Completed!\033[0m')
        return SUCCEED
