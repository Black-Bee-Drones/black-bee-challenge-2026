from datetime import datetime

import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode



from nectar.control import DroneFactory, MavrosConfig, PoseSource
from nectar.vision import ImageHandler

from mapping.config import Config, PoseSourceOption

class Inicialize(State):

    def __init__(self, config: Config):
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config

        self.node = YasminNode.get_instance()

        self.start_time = None

    def execute(self, blackboard: Blackboard):
        yasmin.YASMIN_LOG_INFO('INICIALIZING...')

        #Start Time

        try:
            yasmin.YASMIN_LOG_INFO('Inicializing Start Time...')
            self.start_time = self.node.get_clock().now()

            blackboard.set('Start_time', self.start_time)
            yasmin.YASMIN_LOG_INFO('\033[32mSUCCESSFUL START TIME\033[0m!')

        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'START TIME FAILED: {error}')
            return ABORT

        #Drone

        try:
            yasmin.YASMIN_LOG_INFO(f'Inicializing Drone Config ("{self.config.drone_type}")...')
            if self.config.drone_type != 'mavros':
                yasmin.YASMIN_LOG_INFO('\033[31m Invalid Drone Type (only "mavros" is supported)!\033[0m')
                return ABORT

            pose_source = (
                PoseSource.GPS
                if self.config.pose_source == PoseSourceOption.GPS
                else PoseSource.VISION
            )

            drone_config = MavrosConfig(
                pose_source=pose_source,
                start_driver=self.config.start_driver,
                connection_string=self.config.connection_string,
            )

            drone = DroneFactory.create(self.config.drone_type, drone_config)

            blackboard.set('drone', drone)
            yasmin.YASMIN_LOG_INFO('\033[32mSuccessful Drone Configuration!\033[0m')


        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT

        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'[31mDRONE FACTORY FAILED: {error}')
            return ABORT
        
        #Camera

        try:

            yasmin.YASMIN_LOG_INFO('Initializing Camera(down)...')

            camera_down = ImageHandler(
                image_source=self.config.camera.source,
            )

            yasmin.YASMIN_LOG_INFO('Opening camera (down)...')
            camera_down.open()
            camera_down.run()
            
            yasmin.YASMIN_LOG_INFO('Take testing photo (down)...')
            camera_down.take_photo()
            
            blackboard.set('camera_down', camera_down)
            yasmin.YASMIN_LOG_INFO('\033[32mSuccessful Start Camera(down)!\033[0m')
                
            
                
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT
        
        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'Camera(down) failed: {error}')
            return ABORT

        yasmin.YASMIN_LOG_INFO('\033[32mInicialize Successfully Completed!\033[0m')
        return SUCCEED
