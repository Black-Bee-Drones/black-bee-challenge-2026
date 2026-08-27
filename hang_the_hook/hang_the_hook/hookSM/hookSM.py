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
    RTL
)

class hookSM(StateMachine):
    def __init__(self) -> None:
        super().__init__(outcomes=[SUCCEED, ABORT, RTL])

        self.add_state(
            'FIND_HOSE',
            FindHose(),
            transitions={
                ALIGN: 'ALIGN',
                RTL: RTL
            }
        )

        self.add_state(
            'ALIGN',
            Align(),
            transitions={
                DESCEND: 'DESCEND',
                FIND_HOSE: 'FIND_HOSE',
                RTL: RTL
            }
        )
        self.add_state(
            'DESCEND',
            Descend(),
            transitions={
                FIND_HOSE: 'FIND_HOSE',
                ALIGN: 'ALIGN',
                RTL: RTL,
                SUCCEED: SUCCEED
            }
        )

        self.set_start_state('ALIGN')
