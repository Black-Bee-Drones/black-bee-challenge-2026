import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from nectar.control import MavrosDrone

class Land(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

    def execute(self, blackboard: Blackboard):
        if "drone" not in blackboard:
            yasmin.YASMIN_LOG_ERROR("Drone not available...")
            return ABORT

        drone: MavrosDrone = blackboard["drone"]

        try:
            yasmin.YASMIN_LOG_INFO("LANDING...")

            drone.land()
            drone.delay(2)
            
            yasmin.YASMIN_LOG_INFO("LANDING completed.")

            return SUCCEED

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"LANDING Failed: {e}")
            return ABORT