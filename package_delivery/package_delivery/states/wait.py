import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from package_delivery.constants import Config


class Wait(State):
    def __init__(self, config: Config = Config, ):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config

    def execute(self, blackboard: Blackboard):
        while True:
            try:
                answer = input("Type 'yes' when manually fixed the package: ").strip().lower()
                if answer == "yes":
                    blackboard['has_thePkg'] = True
                    return SUCCEED
                print("Type exactally 'yes' to continue or (Ctrl + C) to ABORT.")
            
            except KeyboardInterrupt:
                yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
                return ABORT
            
            except Exception as e:
                yasmin.YASMIN_LOG_ERROR(f"Waiting failed: {e}")
                return ABORT           
