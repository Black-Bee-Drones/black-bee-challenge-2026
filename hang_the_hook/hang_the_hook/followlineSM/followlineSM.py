from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from hang_the_hook.followlineSM.states import SetupLineDetection, SearchBlueLine, FollowBlueLine

class FollowLineSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "SETUP_LINE_DETECTION",
            SetupLineDetection(),
            transitions={
                SUCCEED: "SEARCH_BLUE_LINE",
                ABORT: ABORT
            }
        )

        self.add_state(
            "SEARCH_BLUE_LINE",
            SearchBlueLine(),
            transitions={
                SUCCEED: "FOLLOW_BLUE_LINE",
                ABORT: "CHECK_END_FOLLOW_LINE"
            }
        )

        self.add_state(
            "FOLLOW_BLUE_LINE",
            FollowBlueLine(),
            transitions={
                SUCCEED: "ALIGN",      ##ver com o marco isso aq dps    
                ABORT: "SEARCH_BLUE_LINE"
            }
        )

        self.set_start_state("SETUP_LINE_DETECTION")
