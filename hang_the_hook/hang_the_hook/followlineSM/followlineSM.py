import rclpy
import yasmin
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers

from states import FollowBlueLine, SearchBlueLine, CheckPoint, EndFL, CheckEndFL, SetupFollowLine

class FollowLineSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "SETUP_FOLLOW_LINE",
            SetupFollowLine(),
            transitions={
                SUCCEED: "LINE_FOLLOW",
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
                SUCCEED: "SEARCH_BLUE_LINE",
                ABORT: ABORT
            }
        )

        self.add_state(
            "CHECK_POINT",
            CheckPoint(),
            transitions={
                SUCCEED: "LINE_FOLLOW",
                ABORT: "CHECK_END_FOLLOW_LINE"
            }
        )

        self.add_state(
            "END_FOLLOW_LINE",
            EndFL(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: "CHECK_END_FOLLOW_LINE"
            }
        )

        self.add_state(
            "CHECK_END_FOLLOW_LINE",
            CheckEndFL(),
            transitions={
                SUCCEED: "END_FOLLOW_LINE",
                ABORT: "LINE_FOLLOW"
            }
        )

        # --- MISSING PART: Define the initial state ---
        self.set_start_state("LINE_FOLLOW")
