from datetime import datetime

import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode



from nectar.control import DroneFactory, MavrosConfig, MavlinkConfig, PoseSource, PIDController
from nectar.vision import ImageHandler, Aruco, ROSConfig, CameraFactory, OpenCVConfig


from mapping import Config

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
            if self.config.drone_type == 'mavros':
                drone_config = MavrosConfig(
                    pose_source=PoseSource.GPS,
                    start_driver=False,
                    connection_string=self.config.conection_string
                )

            elif self.config.drone_type == 'mavlink':
                drone_config = MavlinkConfig(
                    pose_source=PoseSource.GPS,
                    connection_string=self.config.conection_string
                )

            else:
                yasmin.YASMIN_LOG_INFO('\033[31m Invalid Drone Type!\033[0m')
                return ABORT


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
            
            if self.config.down_image_source == "ros":
                cam_config = ROSConfig(topic=self.config.down_ros_topic)
                
            else:
                cam_config = OpenCVConfig(width=self.config.image_width, height=self.config.image_height)
                
            
            camera_down = ImageHandler(
                image_source=self.config.down_image_source,
                config=cam_config
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

    
    
            
