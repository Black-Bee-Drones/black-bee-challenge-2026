from datetime import datetime
import traceback

import yasmin

from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode



from nectar.control import DroneFactory, MavrosConfig, PoseSource, MavlinkConfig
from nectar.vision import ImageHandler, ROSConfig, OpenCVConfig

from mapping.config import Config, PoseSourceOption

import time

class Inicialize(State):

    def __init__(self, config: Config):
        """Args:
            config: Loaded mission configuration; used for drone_type,
                pose_source, start_driver, connection_string, and
                camera.source.
        """
        super().__init__(outcomes=[SUCCEED, ABORT])

        self.config = config

        self.node = YasminNode.get_instance()

        self.start_time = None

    def execute(self, blackboard: Blackboard):
        """Record the mission start time, build the drone connection, and
        initialize/open the down-facing camera.

        Args:
            blackboard: Shared mission state. Writes 'Start_time' (ROS
                clock timestamp), 'drone' (the connected MavrosDrone
                instance), and 'camera_down' (the opened ImageHandler)
                on success.

        Returns:
            SUCCEED once start time, drone, and camera are all set up;
            ABORT if drone_type isn't "mavros", or if an exception/
            KeyboardInterrupt occurs while getting the clock, creating the
            drone, or opening/testing the camera.
        """
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
        # try:
        #     yasmin.YASMIN_LOG_INFO(f'Inicializing Drone Config ("{self.config.drone_type}")...')
        #     if self.config.drone_type != 'mavros':
        #         yasmin.YASMIN_LOG_INFO('\033[31m Invalid Drone Type (only "mavros" is supported)!\033[0m')
        #         return ABORT

        #     pose_source = (
        #         PoseSource.GPS
        #         if self.config.pose_source == PoseSourceOption.GPS
        #         else PoseSource.VISION
        #     )

        #     drone_config = MavrosConfig(
        #         pose_source=pose_source,
        #         start_driver=self.config.start_driver,
        #         connection_string=self.config.connection_string,
        #     )
        
        try:
            
            pose_source = (
                PoseSource.GPS
                if self.config.pose_source == PoseSourceOption.GPS
                else PoseSource.VISION
            )
            
            yasmin.YASMIN_LOG_INFO(f'Inicializing Drone Config ("{self.config.drone_type}")...')
            
            if self.config.drone_type == 'mavros':
                
                drone_config = MavrosConfig(
                    pose_source=pose_source,
                    start_driver=self.config.start_driver,
                    connection_string=self.config.connection_string
                )

            elif self.config.drone_type == 'mavlink':
                drone_config = MavlinkConfig(
                    pose_source=PoseSource.GPS,
                    start_driver=self.config.start_driver,
                    connection_string="/dev/ttyAMA1"
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

            if self.config.sim_mode == True:
                cam_config = ROSConfig(
                    topic=self.config.camera.source,
                    compressed=False
                )
            else:
                cam_config = OpenCVConfig(
                    width=640,
                    height=480,
                    device_index=0
                )
                
        
            camera_down = ImageHandler(
                image_source=self.config.camera.source,
                config=cam_config,
            
            )
            

            yasmin.YASMIN_LOG_INFO('Opening camera (down)...')
            camera_down.open()
            # camera_down.run()
            time.sleep(2)

            
            
            yasmin.YASMIN_LOG_INFO('Take testing photo (down)...')
           
            frame = camera_down.take_photo(timeout_sec=15)
            if frame is None:
                yasmin.YASMIN_LOG_WARN("Failed to get frame from camera.")
                return ABORT
            
            blackboard.set('camera_down', camera_down)
            yasmin.YASMIN_LOG_INFO('\033[32mSuccessful Start Camera(down)!\033[0m')
                
            
                
        except KeyboardInterrupt:
            yasmin.YASMIN_LOG_WARN('Execution interrupted by user!')
            return ABORT
        
        except Exception as error:
            yasmin.YASMIN_LOG_ERROR(f'Camera(down) failed: {error}')
            yasmin.YASMIN_LOG_INFO(traceback.format_exc())
            return ABORT

        yasmin.YASMIN_LOG_INFO('\033[32mInicialize Successfully Completed!\033[0m')
        return SUCCEED
