import rclpy
from yasmin import State, StateMachine, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers
from yasmin_viewer import YasminViewerPub

from core.states import(
    Initialize,
    Takeoff,
#    End,
#    ReturnToLaunch
)

# from hookSM.states import(
#    Align,
#    Descend,
#    Hook,
#    EndHook,
#)

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

def main():
    rclpy.init()

    set_ros_loggers()

    hooktest_sm = HookTest()
    hooktest_sm.set_sigint_handler(True)
    viewer = YasminViewerPub(hooktest_sm, "HOOKTEST_FSM")

    try:
        outcome = hooktest_sm()
        print (f"State machine finished with status: {outcome}")

    except KeyboardInterrupt:
        print("Stopping by keyboard interrupt...")
        hooktest_sm.cancel_state()

    except Exception as e:
        print(f"State machine finished with exception: {e}")

    finally:
        if hasattr(viewer, "shutdown"):
            viewer.shutdown()  # type: ignore

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == "__main__":
    main()
