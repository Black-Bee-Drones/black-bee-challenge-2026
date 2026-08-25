import rclpy

import nectar

from traceback import print_exc

from yasmin import StateMachine, Blackboard
from yasmin_ros import set_ros_loggers as yasmin_set_ros_loggers
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

from hang_the_hook.core.states import Initialize, Takeoff, ReturnToLaunch, End
from hang_the_hook.followlineSM.followlineSM import FollowLineSM
from hang_the_hook.hookSM.hookSM import hookSM

class HangTheHookSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={
                SUCCEED: "TAKEOFF",
                ABORT: "END"
            }
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={
                SUCCEED: "FOLLOW_LINE",
                ABORT: "END"
            }
        )

        self.add_state(
            "FOLLOW_LINE",
            FollowLineSM(),
            transitions={
                SUCCEED: "HOOK",
                ABORT: "RETURN_TO_LAUNCH",
            }
        )

        self.add_state(
            "HOOK",
            hookSM(),
            transitions={
                SUCCEED: "RETURN_TO_LAUNCH",
                ABORT: "RETURN_TO_LAUNCH"
            }
        )

        self.add_state(
            'RETURN_TO_LAUNCH',
            ReturnToLaunch(),
            transitions={
                SUCCEED: 'END',
            }
        )
        self.add_state(
            "END",
            End(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: ABORT
            }
        )

        self.set_start_state("INITIALIZE")

'''
YASMIN VIEWER SETUP

Install dependency (ubuntu / debian):
sudo apt install ros-$ROS_DISTRO-yasmin ros-$ROS_DISTRO-yasmin-ros ros-$ROS_DISTRO-yasmin-viewer

Run viewer node:
ros2 run yasmin_viewer yasmin_viewer_node

Access web visualization:
http://localhost:5000/

Tip:
To run node across different IP addresses using ros parameters
ros2 run yasmin_viewer yasmin_viewer_node --ros-args -p host:=127.0.0.1 -p port:=5032
'''

def mangalarga():

    try:
        rclpy.init()
        yasmin_set_ros_loggers()

        nectar.use_executor(YasminNode.get_instance()._executor)

        mangalarga_sm = HangTheHookSM()

        # Initialize a fresh Blackboard specifically for the mangalarga sequence
        mangalarga_blackboard = Blackboard()

        # Keep the viewer alive, even though the object is never called
        _ = YasminViewerPub(mangalarga_sm, "MANGALARGA_FSM")

        outcome = mangalarga_sm(blackboard=mangalarga_blackboard)
        print (f"Mangalarga finished with status: {outcome}")

    except KeyboardInterrupt:
        print("Stopping by keyboard interrupt...")
        mangalarga_sm.cancel_state()

    except Exception as e:
        print(f"Mangalarga finished with exception: {e}")
        print_exc()

    finally:
        if nectar.is_initialized():
            nectar.shutdown()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    mangalarga()
