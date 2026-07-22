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
    
    hang_the_hook_sm = HangTheHookSM()
    
    try:
        hang_the_hook_sm.execute()
    except KeyboardInterrupt:
        print("Parando por interrupção de teclado...")
        if hang_the_hook_sm.is_running():
            hang_the_hook_sm.cancel_state()   
            
    if rclpy.ok():
        rclpy.shutdown()
        
if __name__ == "__main__":
    main()     