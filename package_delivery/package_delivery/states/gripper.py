import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from nectar.control import MavrosDrone, MavlinkDrone, MoveReference
from package_delivery.constants import Config

class Gripper(State):
    def __init__(self, target_has_pkg: bool, config: Config = Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.config = config
        self.target_has_pkg = target_has_pkg

    def execute(self, blackboard: Blackboard):
        if self.config.drone_type == 'mavros':
            drone : MavrosDrone = blackboard.get('drone')

        elif self.config.drone_type == 'mavlink':
            drone : MavlinkDrone = blackboard.get('drone')
        
        else:
            yasmin.YASMIN_LOG_ERROR("Drone Type (MavrosDrone or MavlinkDrone) Not Find")
            return ABORT 
        
        if 'has_thePkg' not in blackboard:
            yasmin.YASMIN_LOG_ERROR('Flag - has the pkg - not found.')
            return ABORT
        
        pwm = (self.config.servo_closed_pwm if self.target_has_pkg else self.config.servo_open_pwm)

        try:
            yasmin.YASMIN_LOG_INFO(f'Set {pwm} pwm value')
            
            if not drone.do_servo(aux_out=self.config.servo_channel,pwm_value=pwm):
                yasmin.YASMIN_LOG_ERROR('Failed to do servo.')
                return ABORT
            drone.delay(self.config.servo_action_delay)

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user.')
            return ABORT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f'Gripper failed: {e}')
            return ABORT
        blackboard['has_thePkg'] = self.target_has_pkg
        yasmin.YASMIN_LOG_INFO('Completed successfully.')
        return SUCCEED
    