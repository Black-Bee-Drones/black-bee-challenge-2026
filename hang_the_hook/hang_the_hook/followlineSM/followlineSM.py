from hang_the_hook.followlineSM.constants import FOUND_RED, FOUND_BLUE, SEARCH
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT

from hang_the_hook.followlineSM.states import SearchBlueLine, FollowBlueLine

class FollowLineSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "SEARCH_BLUE_LINE",
            SearchBlueLine(),
            transitions={
                FOUND_BLUE: "FOLLOW_BLUE_LINE",
                FOUND_RED: SUCCEED,  # Transition to SUCCEED if red line is found
                ABORT: ABORT,
            }
        )

        self.add_state(
            "FOLLOW_BLUE_LINE",
            FollowBlueLine(),
            transitions={
                SEARCH: "SEARCH_BLUE_LINE", 
                ABORT: ABORT,
            }
        )

        self.set_start_state("SEARCH_BLUE_LINE")
