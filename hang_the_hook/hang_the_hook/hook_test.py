import rclpy

from yasmin import StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from core.states import(
    Initialize,
    Takeoff,
    End,
    ReturnToLaunch
)

from hookSM.states import(
    Align,
    Descend,
    Hook,
    EndHook,
)

class HookTest(StateMachine):

    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, "END"])

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
                SUCCEED: SUCCEED,
                ABORT: ABORT
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

def main():
    rclpy.init()

    set_ros_loggers()

    hooktest_sm = HookTest()

    # Intercept the ctrl+c signal to cleanly shutdown the state machine
    hooktest_sm.set_sigint_handler(True)

    # Initialize a fresh Blackboard specifically for the hook test sequence
    hook_blackboard = Blackboard()

    # Keep the viewer alive, even though the object is never called
    _ = YasminViewerPub(hooktest_sm, "HOOKTEST_FSM")

    try:
        outcome = hooktest_sm(blackboard=hook_blackboard)
        print (f"State machine finished with status: {outcome}")

    except KeyboardInterrupt:
        print("Stopping by keyboard interrupt...")
        hooktest_sm.cancel_state()

    except Exception as e:
        print(f"State machine finished with exception: {e}")

    finally:
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
