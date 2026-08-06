import time

import yasmin
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT

from mapping.states import(
    Inicialize,
    Takeoff,
    Land
)

from mapping import Config


class MappingSM(StateMachine):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])
        
        self.add_state(
            'INICIALIZE',
            Inicialize(config),
            transitions={SUCCEED: 'TAKEOFF', ABORT:ABORT}
        )
        
        self.add_state(
            'TAKEOFF',
            Takeoff(config),
            transitions={SUCCEED:'LAND', ABORT: ABORT}
        )
        
        self.add_state(
            'LAND',
            Land(config),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT}
        )

        self.set_start_state('INICIALIZE')