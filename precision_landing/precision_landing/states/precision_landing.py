import math
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT
from yasmin_ros.yasmin_node import YasminNode

from rclpy.duration import Duration

from nectar.vision import ImageHandler
from nectar.control import MavrosDrone, PIDController
from precision_landing.constants import (
    IMAGE_WIDTH,
    IMAGE_HEIGHT,
    PRECISION_LANDING_TIME,
    CONTROLER_P_XY,
    CONTROLER_I_XY,
    CONTROLER_D_XY,
    CONTROLER_OUTPUT_LIMITS_XY,
    CONTROLER_INTEGRAL_LIMITS_XY,
    CONTROLER_P_Z,
    CONTROLER_I_Z,
    CONTROLER_D_Z,
    CONTROLER_OUTPUT_LIMITS_Z,
    CONTROLER_INTEGRAL_LIMITS_Z,
    PRECISE_DOWN_TOLERANCE_PX
)

class Precision_landing(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT])
        self.node = YasminNode.get_instance()
        
        self.pid_x = PIDController(
            kp=CONTROLER_P_XY,
            ki=CONTROLER_I_XY,
            kd=CONTROLER_D_XY,
            output_limits=CONTROLER_OUTPUT_LIMITS_XY,
            integral_limits=CONTROLER_INTEGRAL_LIMITS_XY,
        )

        self.pid_y = PIDController(
            kp=CONTROLER_P_XY,
            ki=CONTROLER_I_XY,
            kd=CONTROLER_D_XY,
            output_limits=CONTROLER_OUTPUT_LIMITS_XY,
            integral_limits=CONTROLER_INTEGRAL_LIMITS_XY,
        )

        self.pid_z = PIDController(
            kp=CONTROLER_P_Z,
            ki=CONTROLER_I_Z,
            kd=CONTROLER_D_Z,
            output_limits=CONTROLER_OUTPUT_LIMITS_Z,
            integral_limits=CONTROLER_INTEGRAL_LIMITS_Z,
        )

    def PIXEL_POR_METRO (self, altitude_m: float, fov_deg: float, width: float):
            #calcular FOV, muito importante para detecção correta
            half_fov_rad = math.radians(fov_deg/2.0)
            return width / (2.0 * altitude_m * math.tan(half_fov_rad))


    def execute(self, blackboard: Blackboard):
        try:
            if "drone" not in blackboard:
                yasmin.YASMIN_LOG_ERROR("Drone not available...")
                return ABORT
             
            drone: MavrosDrone  = blackboard["drone"]
             
            if "camera" not in blackboard:
                yasmin.YASMIN_LOG_ERROR("CAMERA NOT AVAILABLE... ABORTING")
                return ABORT
             
            camera: ImageHandler = blackboard["camera"]
            camera.open()
             
            start_time = self.node.get_clock().now() #gets the start time of the state
            precision_landing_time = Duration(seconds=PRECISION_LANDING_TIME) #gets the max time in seconds before TIMEOUT
             
            while (self.node.get_clock.now() - start_time) < precision_landing_time:

                frame = camera.take_photo()
             
                for shape in frame.filter_by_class([blackboard["aruco_shape"]]):
                    for number in frame.filter_by_class([blackboard["aruco_id"]]):

                        if(
                        abs(shape.center[0] - number.center[0]) <= shape.width/2 
                        and 
                        abs(shape.center[1] - number.center[1]) <= shape.heigth/2):
             
                            target_x = shape.center[0]
                            target_y = shape.center[1]
             
                            erro_x_pixel = target_x - IMAGE_WIDTH/2
                            erro_y_pixel = target_y - IMAGE_HEIGHT/2
             
                            erro_x = erro_x_pixel / self.PIXEL_POR_METRO(drone.get_altitude(), 86, IMAGE_WIDTH/2)
                            erro_y = erro_y_pixel / self.PIXEL_POR_METRO(drone.get_altitude(), 47, IMAGE_HEIGHT/2)
                            erro_z = drone.get_altitude() - 0.7                        
             
                            output_x = self.pid_x.update(erro_x)
                            output_y = self.pid_y.update(erro_y)
                            output_z = self.pid_z.update(erro_z)
                                    
                            drone.move_velocity(
                                vx = output_x,
                                vy = output_y,
                                vz = output_z if (abs(erro_x_pixel) <= PRECISE_DOWN_TOLERANCE_PX and abs(erro_y_pixel) <= PRECISE_DOWN_TOLERANCE_PX) else 0.0,
                                vyaw = 0.0,
                            )
                            """
                            if(shape.width >= TAMANHO_DE_LOSE):
                                drone.land()
                                return SUCEED
                            
                            """
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"PRECISION_LANDING Failed: {e}")
            return ABORT

