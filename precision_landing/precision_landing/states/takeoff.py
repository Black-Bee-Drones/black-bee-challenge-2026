import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone

from constants import (
    TAKEOFF_HEIGHT,
)

class Takeoff(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not available...")
            return ABORT

        drone: MavrosDrone = blackboard["drone"]

        try:
            yasmin.YASMIN_LOG_INFO("Starting TAKEOFF...")

            drone.set_home() #Sets the current gps position as 'home'
            drone.arm()
            drone.takeoff(TAKEOFF_HEIGHT, max_retries=3, timeout=30.0, precision=0.2)
            drone.delay(1)
            
            yasmin.YASMIN_LOG_INFO("TAKEOFF completed.")

            return SUCCEED
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"TAKEOFF Failed: {e}")
            return ABORT