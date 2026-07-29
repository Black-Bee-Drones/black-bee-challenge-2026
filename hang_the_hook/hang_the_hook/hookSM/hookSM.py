import time
import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from states import(
    Align,
    Descend,
    Hook,
    EndHook,
)

class hookSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "ALIGN",
            Align(),
            transitions={
                SUCCEED: "DESCEND",
                ABORT: ABORT
            }
        )
        self.add_state(
            "DESCEND",
            Descend(),
            transitions={
                SUCCEED: "DROP_HOOK",
                ABORT: "ALIGN"
            }
        )
        self.add_state(
            "DROP_HOOK",
            Hook(),
            transitions={
                SUCCEED: "END_HOOK",
                ABORT: "ALIGN"
            }
        )
        self.add_state(
            "END_HOOK",
            EndHook(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT
            }
        )

        self.set_start_state("ALIGN")
