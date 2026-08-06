from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT

from .states import Search, FindTargetBase

#from states import

class FindSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, TIMEOUT])

        self.add_state(
            "SEARCH",
            Search(),
            transitions={SUCCEED: "FIND_TARGET_BASE", ABORT: ABORT, TIMEOUT: TIMEOUT}
        )

        self.add_state(
            "FIND_TARGET_BASE",
            FindTargetBase(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT, TIMEOUT: TIMEOUT}
        )

        self.set_start_state("SEARCH")