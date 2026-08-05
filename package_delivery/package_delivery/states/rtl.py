import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from nectar.control import MavlinkDrone, RTLMethod

from package_delivery.constants import Config

class Rtl(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        
        self.config = config
    
    def execute(self, bb: Blackboard):
        
        self.drone: MavlinkDrone = bb["drone"]

        yasmin.YASMIN_LOG_INFO("RTL...")
        try:
            self.drone.rtl(altitude=self.config.rtl_altitude, RTLMethod=RTLMethod.NAVIGATE, land=False).wait(timeout=30)
            yasmin.YASMIN_LOG_INFO("Return to launch initiated.")
            return SUCCEED
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"RTL failed: {e}")
            return ABORT