from rclpy import(
    init as rclpy_init,
    ok as rclpy_ok,
    shutdown as rclpy_shutdown,
)

from nectar import(
    init as nectar_init,
    is_initialized as nectar_ok,
    shutdown as nectar_shutdown,
)

from traceback import print_exc

from yasmin import StateMachine, Blackboard
from yasmin_ros import set_ros_loggers as yasmin_set_ros_loggers
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_viewer import YasminViewerPub

from hang_the_hook.core.states import Initialize, Takeoff, ReturnToLaunch, End
from hang_the_hook.followlineSM.followlineSM import FollowLineSM
# TODO: uncomment when hookSM is ready
# from hang_the_hook.hookSM.hookSM import hookSM

class HangTheHookSM(StateMachine):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.add_state(
            "INITIALIZE",
            Initialize(),
            transitions={
                SUCCEED: "TAKEOFF",
                ABORT: ABORT
            }
        )

        self.add_state(
            "TAKEOFF",
            Takeoff(),
            transitions={
                SUCCEED: "FOLLOW_LINE",
                ABORT: ABORT
            }
        )

        self.add_state(
            "FOLLOW_LINE",
            FollowLineSM(),
            transitions={
                # TODO: change back to "HOOK" when hookSM is ready
                SUCCEED: "RETURN_TO_LAUNCH",
                ABORT: ABORT,
            }
        )

        # TODO: uncomment when hookSM is ready
        # self.add_state(
        #     "HOOK",
        #     hookSM(),
        #     transitions={
        #         SUCCEED: "RETURN_TO_LAUNCH",
        #         ABORT: ABORT
        #     }
        # )

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
        rclpy_init()
        nectar_init()

        yasmin_set_ros_loggers()

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
        if nectar_ok():
            nectar_shutdown()

        if rclpy_ok():
            rclpy_shutdown()

if __name__ == "__main__":
    mangalarga()
