from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, FAIL, TIMEOUT

from .states import Search, FindTargetBase

class FindSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, TIMEOUT])

        self.add_state(
            "SEARCH",
            Search(),
            transitions={SUCCEED: "FIND_TARGET_BASE", ABORT: ABORT, TIMEOUT: TIMEOUT, FAIL: "SEARCH"}
        )

        self.add_state(
            "FIND_TARGET_BASE",
            FindTargetBase(),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT, TIMEOUT: TIMEOUT, FAIL: "FIND_TARGET_BASE"}
        )

        self.set_start_state("SEARCH")