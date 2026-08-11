import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from nectar.control import MavrosDrone, MavlinkDrone, RTLMethod

from package_delivery.constants import Config

class Rtl(State):
    def __init__(self, config: Config):
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

        yasmin.YASMIN_LOG_INFO("RTL...")
        try:
            self.drone.rtl(altitude=self.config.rtl_altitude, RTLMethod=RTLMethod.NAVIGATE, land=True)
            yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT