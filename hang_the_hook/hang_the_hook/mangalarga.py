import rclpy
import yasmin
from yasmin import StateMachine
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros import set_ros_loggers

from core.states import Initialize, Takeoff, End, RTL
from followlineSM import FollowLineSM
from hookSM import hookSM

class HangTheHookSM(StateMachine):
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
                SUCCEED: "FOLLOW_LINE",
                ABORT: ABORT
            }
        )

        self.add_state(
            "FOLLOW_LINE",
            FollowLineSM(),
            transitions={
                SUCCEED: "HOOK",
                ABORT: ABORT
            }
        )

        self.add_state(
            "HOOK",
            hookSM(),
            transitions={
                SUCCEED: "RTL",
                ABORT: ABORT
            }
        )

        self.add_state(
            "RTL",
            RTL(),
            transitions={
                SUCCEED: "END",
                ABORT: ABORT
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

def main():
    rclpy.init()

    set_ros_loggers()

    mangalarga_sm = HangTheHookSM()
    viewer = yasmin.visualization.StateMachineViewer(mangalarga_sm)
    viewer.start()

    try:
        status = mangalarga_sm()
        print (f"State machine finished with status: {status}")

    except KeyboardInterrupt:
        print("Stopping by keyboard interrupt...")
        try:
            mangalarga_sm.set_outcome("END")
        except KeyError:
            print("Drone not initialized yet, nothing to land.")

    except Exception as e:
        print(f"State machine finished with exception: {e}")
        mangalarga_sm.set_outcome("END")

    finally:
        viewer.stop()
        rclpy.shutdown()

if __name__ == "__main__":
    main()