from datetime import datetime

import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode



from nectar.control import DroneFactory, MavrosConfig, MavlinkConfig, PoseSource, PIDController
from nectar.vision import ImageHandler, Aruco, ROSConfig


from config import Config

class Inicialize(State):
    
    def __int__(self, config : Config):
        super().__init__(outcomes=[SUCCEED, ABORT])
        
        self.config = config
        
        self.node = YasminNode.get_instance()
        
        self.start_time = None
        
    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO('INICIALIZING...')
        
        #Start Time
        
        try:
            yasmin.YASMIN_LOG_INFO('    Inicializing Start Time...')
            self.start_time = self.node.get_clock().now()
            
            blackboard.set('Start_time', self.start_time)
            yasmin.YASMIN_LOG_INFO('    \033[32mSUCCESSFUL START TIME\033[0m!')
            
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_INFO('    \033[31mExecution interrupted by user!\033[0m')
            return ABORT
        
        except Exception as error:
            yasmin.YASMIN_LOG_INFO(f'   \033[31mSTART TIME FAILED: {error}\033[0m')
            return ABORT
        
        #Drone 
        
        try: 
            yasmin.YASMIN_LOG_INFO(f'    Inicializing Drone Config ("{self.config.drone_type}")...')
            if self.config.drone_type == 'mavros':
                drone_config = MavrosConfig(
                    pose_source=PoseSource.VISION,
                    connection_string=self.config.conection_string
                )
            
            elif self.config.drone_type == 'mavlink':
                drone_config = MavlinkConfig(
                    pose_source=PoseSource.VISION,
                    connection_string=self.config.conection_string
                )
            
            else:
                yasmin.YASMIN_LOG_INFO('    \033[31m Invalid Drone Type!\033[0m')
                return ABORT

            
            drone = DroneFactory.create(self.config.drone_type, drone_config)
            
            blackboard.set('drone', drone)
            yasmin.YASMIN_LOG_INFO('    \033[32mSuccessful Drone Configuration!\033[0m')
            
            
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_INFO('    \033[31mExecution interrupted by user!\033[0m')
            return ABORT
        
        except Exception as error:
            yasmin.YASMIN_LOG_INFO(f'   \033[31mSTART TIME FAILED: {error}\033[0m')
            return ABORT
        
        
        