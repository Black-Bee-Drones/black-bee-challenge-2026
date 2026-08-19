import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone

from mapping.config import Config, LandingMode


class Land(State):

    def __init__(self, config : Config):
        """Args:
            config: Loaded mission configuration; used for drone_type and
                landing_mode.
        """
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config


    def execute(self, blackboard: Blackboard):
        """Land the drone, via RTL or a direct LAND command depending on
        config.landing_mode.

        Args:
            blackboard: Shared mission state. Reads 'drone' (the connected
                MavrosDrone set by Inicialize).

        Returns:
            SUCCEED once the land/RTL command completes; ABORT if
            drone_type isn't "mavros", or if an exception/
            KeyboardInterrupt occurs while landing.
        """

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
