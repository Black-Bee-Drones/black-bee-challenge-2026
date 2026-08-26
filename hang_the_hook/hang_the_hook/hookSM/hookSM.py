from yasmin import StateMachine
from yasmin_ros.basic_outcomes import ABORT, SUCCEED

from hang_the_hook.hookSM.states import(
    FindHose,
    Align,
    Descend,
)
from hang_the_hook.hookSM.constants import(
    FIND_HOSE,
    ALIGN,
    DESCEND,
)

class hookSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "FIND_HOSE",
            FindHose(),
            transitions={
                FIND_HOSE: 'FIND_HOSE',
                ALIGN: 'ALIGN',
                ABORT: ABORT
            }
        )

        self.add_state(
            "ALIGN",
            Align(),
            transitions={
                DESCEND: 'DESCEND',
                ABORT: ABORT
            }
        )
        self.add_state(
            "DESCEND",
            Descend(),
            transitions={
                FIND_HOSE: 'FIND_HOSE',
                ALIGN: 'ALIGN',
                ABORT: ABORT,
                SUCCEED: SUCCEED
            }
        )

        self.set_start_state('ALIGN')
