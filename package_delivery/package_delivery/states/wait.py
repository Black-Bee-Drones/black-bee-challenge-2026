import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from package_delivery.constants import Config


class Wait(State):
    def __init__(self, config: Config = Config, ):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        try:
            if (input("Type 'yes' when manually fixed the package: ") == "yes"):
                return SUCCEED
            else:
                return ABORT
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"Waiting failed: {e}")
            return ABORT            
