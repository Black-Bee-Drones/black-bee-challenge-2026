import rclpy

import yasmin
from yasmin_ros.yasmin_node import YasminNode
from yasmin_ros.basic_outcomes import SUCCEED
from yasmin_ros import set_ros_loggers

import nectar


from mapping import (
    Config,
    #SITLConfig,
    MappingSM
    
)



def main(args=None):
    
    #config = SITLConfig()
    config = Config.load()
    
    rclpy.init(args=args)
    set_ros_loggers()
    
    nectar.use_executor(YasminNode.get_instance()._executor)
    
    mapping_sm = None
    
    try:
        yasmin.YASMIN_LOG_INFO(f'Inicializing the State Machine...')
        
        mapping_sm = MappingSM(config)
        final_outcome = mapping_sm()
        
        
    except Exception as error:
        yasmin.YASMIN_LOG_ERROR(f'Mapping State Machine Failed: {error}!')
        
    else:
        if final_outcome == SUCCEED:
            yasmin.YASMIN_LOG_INFO(final_outcome)
        else:
            yasmin.YASMIN_LOG_ERROR(final_outcome)
            
    nectar.shutdown()
    
    if rclpy.ok():
        rclpy.shutdown()
        
    
if __name__ == '__main__':
    main()
        
        