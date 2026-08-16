import yasmin
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, CANCEL, ABORT, TIMEOUT

from mapping.states import(
    Inicialize,
    Takeoff,
    Land,
    PlanCoverage,
    CaptureWaypoint,
    CAPTURE_NEXT,
    DetectBases,
    PublishResults,
)

from mapping import Config


class MappingSM(StateMachine):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT, CANCEL, TIMEOUT])

        self.add_state(
            'INICIALIZE',
            Inicialize(config),
            transitions={SUCCEED: 'TAKEOFF', ABORT: ABORT}
        )

        self.add_state(
            'TAKEOFF',
            Takeoff(config),
            transitions={SUCCEED: 'PLAN_COVERAGE', ABORT: ABORT}
        )

        self.add_state(
            'PLAN_COVERAGE',
            PlanCoverage(config),
            transitions={SUCCEED: 'CAPTURE_WAYPOINT', ABORT: ABORT}
        )

        self.add_state(
            'CAPTURE_WAYPOINT',
            CaptureWaypoint(config),
            transitions={
                CAPTURE_NEXT: 'CAPTURE_WAYPOINT',
                SUCCEED: 'DETECT_BASES',
                ABORT: ABORT,
            }
        )

        self.add_state(
            'DETECT_BASES',
            DetectBases(config),
            transitions={SUCCEED: 'PUBLISH_RESULTS', ABORT: ABORT}
        )

        self.add_state(
            'PUBLISH_RESULTS',
            PublishResults(config),
            transitions={SUCCEED: 'LAND', ABORT: ABORT}
        )

        self.add_state(
            'LAND',
            Land(config),
            transitions={SUCCEED: SUCCEED, ABORT: ABORT}
        )

        self.set_start_state('INICIALIZE')
