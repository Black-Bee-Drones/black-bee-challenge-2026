import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone, MavlinkConfig

from mapping.config import Config

class Takeoff(State):

    def __init__(self, config : Config):
        """Args:
            config: Loaded mission configuration; used for drone_type and
                takeoff_altitude.
        """
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config


    def execute(self, blackboard: Blackboard):
        """Set home, arm, and take off to config.takeoff_altitude.

        Args:
            blackboard: Shared mission state. Reads 'drone' (the connected
                MavrosDrone set by Inicialize).

        Returns:
            SUCCEED once set_home/arm/takeoff complete and the post-takeoff
            settle delay elapses; ABORT if drone_type isn't "mavros", or if
            an exception/KeyboardInterrupt occurs during arming/takeoff.
        """

        # if self.config.drone_type != 'mavros':
        #     yasmin.YASMIN_LOG_INFO('\033[31mDrone Type Not Found (only MavrosDrone is supported)!\033[0m')
        #     return ABORT

        drone: MavrosDrone | MavlinkConfig = blackboard.get('drone')


        yasmin.YASMIN_LOG_INFO(f'Taking off to altitude: {self.config.takeoff_altitude} m ... ')

        try:
            drone.takeoff(self.config.takeoff_altitude,adjust_altitude=False)
            drone.delay(3)
            drone.move_to(x=0.0,y=0.0,z=(3-drone.get_altitude()))

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_INFO('    \033[31mExecution interrupted by user!\033[0m')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_INFO(f'   \033[31mTAKEOFF FAILED: {error}\033[0m')
            return ABORT

        yasmin.YASMIN_LOG_INFO('    \033[32mTakeoff Successfully Completed!\033[0m')
        return SUCCEED
