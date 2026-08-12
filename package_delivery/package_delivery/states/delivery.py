import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from nectar.control import MavrosDrone, MavlinkDrone, MoveReference
from package_delivery.constants import Config

class Delivery(State):
    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config
        self.has_thePkg = config.has_thePkg
    
    def execute(self, blackboard: Blackboard):
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
        
        else:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT 
        
        
        pwm = (self.config.servo_closed_pwm if self.has_thePkg else self.config.servo_open_pwm)

        try:
            yasmin.YASMIN_LOG_INFO(f'Set {pwm} pwm value')
            
            if not drone.do_servo(aux_out=self.config.servo_channel,pwm_value=pwm):
                yasmin.YASMIN_LOG_ERROR('Failed to do servo.')
                return ABORT

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Gripper failed: {e}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED