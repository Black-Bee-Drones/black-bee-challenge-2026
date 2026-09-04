import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone

from precision_landing.constants import (
    TAKEOFF_HEIGHT,
)

class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not available...")
            return ABORT

        drone: MavrosDrone  = blackboard["drone"]

        try:
            yasmin.YASMIN_LOG_INFO("Starting TAKEOFF...")

            try:
                drone.takeoff(TAKEOFF_HEIGHT, max_retries=5, timeout=30.0, precision=0.2)
            except:
                yasmin.YASMIN_LOG_INFO("TAKEOFF FAILED... ABORTING")
                return ABORT
            drone.delay(2)
            
            yasmin.YASMIN_LOG_INFO("TAKEOFF completed.")

            return SUCCEED

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_INFO("ABORTING")
            return ABORT
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"TAKEOFF Failed: {e}")
            return ABORT