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

from hang_the_hook.core.states import(
    Initialize,
    Takeoff,
    End,
    ReturnToLaunch
)

from hang_the_hook.hookSM.states import(
    Align,
    Descend,
    Hook,
    EndHook,
)

class HookTest(StateMachine):

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
                SUCCEED: "RETURN_TO_LAUNCH",
                ABORT: ABORT
            }
        )

        self.add_state(
            "RETURN_TO_LAUNCH",
            ReturnToLaunch(),
            transitions={
                SUCCEED: "END",
                ABORT: "RETURN_TO_LAUNCH"
            }
        )

        self.add_state(
            "END",
            End(),
            transitions={
                SUCCEED: SUCCEED,
                ABORT: "END"
            }
        )
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

def hook_test():

    try:
        rclpy_init()
        nectar_init()

        yasmin_set_ros_loggers()

        hook_test_sm = HookTest()

        # Initialize a fresh Blackboard specifically for the hook_test sequence
        hook_test_blackboard = Blackboard()

        # Keep the viewer alive, even though the object is never called
        _ = YasminViewerPub(hook_test_sm, "HOOK_TEST_FSM")

        outcome = hook_test_sm(blackboard=hook_test_blackboard)
        print (f"Hook_test finished with status: {outcome}")

    except KeyboardInterrupt:
        print("Stopping by keyboard interrupt...")
        hook_test_sm.cancel_state()

    except Exception as e:
        print(f"Hook_test finished with exception: {e}")
        print_exc()

    finally:
        if nectar_ok():
            nectar_shutdown()

        if rclpy_ok():
            rclpy_shutdown()

if __name__ == "__main__":
    hook_test()
