import math
import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, ABORT, TIMEOUT
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
    PRECISE_DOWN_TOLERANCE_PX,
    MAX_ALTITUDE,
    FINAL_LANDING_TOLERANCE,
    FINAL_LANDING_HEIGHT,
)





class Precision_landing(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, TIMEOUT])
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
            #PIXEL PER METER CALCULATION
            #NOTE verificar se fov está certo
            half_fov_rad = math.radians(fov_deg/2.0)
            return width / (2.0 * altitude_m * math.tan(half_fov_rad))

    def REGAIN_TARGET(self):
        print("ola")


    def execute(self, blackboard: Blackboard):
        output_x = 0
        output_y = 0
        try:
            if "drone" not in blackboard:
                yasmin.YASMIN_LOG_ERROR("Drone not available...")
                return ABORT
             
            drone: MavrosDrone  = blackboard["drone"]
             
            if "camera" not in blackboard:
                yasmin.YASMIN_LOG_ERROR("CAMERA NOT AVAILABLE... ABORTING")
                return ABORT
             
            camera: ImageHandler = blackboard["camera"]
             
            start_time = self.node.get_clock().now() #gets the start time of the state
            precision_landing_time = Duration(seconds=PRECISION_LANDING_TIME) #gets the max time in seconds before TIMEOUT
             
            while (self.node.get_clock().now() - start_time) < precision_landing_time:

                result = camera.take_photo()
                TARGET_FOUND = False
             
                for shape in result.filter_by_class([blackboard["aruco_shape"]]):
                    for number in result.filter_by_class([blackboard["aruco_id"]]):

                        if(
                        abs(shape.center[0] - number.center[0]) <= shape.width/2 
                        and 
                        abs(shape.center[1] - number.center[1]) <= shape.height/2):

                            TARGET_FOUND = True
                            h, w = result.image.shape[:2]
                            target_x = shape.center[1]
                            target_y = shape.center[0]
             
                            erro_x_pixel = target_x - w/2
                            erro_y_pixel = target_y - h/2
             
                            erro_x = erro_x_pixel / self.PIXEL_POR_METRO(drone.get_altitude(), 60, IMAGE_WIDTH)#HFOV
                            erro_y = erro_y_pixel / self.PIXEL_POR_METRO(drone.get_altitude(), 47, IMAGE_HEIGHT)#VFOV
                            erro_z = drone.get_altitude() - FINAL_LANDING_HEIGHT                       
             
                            output_x = self.pid_x.update(erro_x)
                            output_y = self.pid_y.update(erro_y)
                            output_z = -0.5 if (abs(erro_x_pixel) <= PRECISE_DOWN_TOLERANCE_PX and abs(erro_y_pixel) <= PRECISE_DOWN_TOLERANCE_PX and erro_z >= 0) else 0.0
                            yasmin.YASMIN_LOG_INFO(f'Detection at: error_x={erro_x:.2f}, error_x_px={erro_x_pixel:.2f}, error_y={erro_y:.2f}, error_y_px={erro_y_pixel:.2f} output_x={output_x:.2f}, output_y={output_y:.2f}, drone_h={drone.get_altitude()}')
                            drone.move_velocity(
                                vx = output_x,  
                                vy = output_y,
                                vz = output_z,
                                vyaw = 0.0,
                            )
                            drone.delay(0.4)
                            #THIS BREAK IS IN CASE THERE ARE MORE OF THE ANSWERS IN THE PITURE
                            break

                if not TARGET_FOUND:
                    alt = drone.get_altitude()

                    if (alt <= FINAL_LANDING_HEIGHT) and (abs(erro_x) <= FINAL_LANDING_TOLERANCE and abs(erro_y) <= FINAL_LANDING_TOLERANCE):
                        #TAKES THE LAND WHEN DETECTION IS LOST
                        yasmin.YASMIN_LOG_INFO("TAKING LAST LAND")
                        drone.move_velocity(vx=0,vy=0,vz=0)
                        drone.delay(1)
                        drone.land()
                        return SUCCEED

                    elif alt < MAX_ALTITUDE:
                        #TRY TO WAIT AFTER DONT FINDING THE TARGET
                        yasmin.YASMIN_LOG_INFO("TARGET LOST... WAITING")
<<<<<<< HEAD
                        drone.move_velocity(vx=output_x,vy=output_y, vz = output_z)
                    drone.delay(0.5)

=======
                        drone.move_velocity(vx=0,vy=0,vz=0)
                        drone.move_to(z=MAX_ALTITUDE)
                    else:
                        #NÃO SEI OQUE FAZER AQUI
                        drone.move_velocity(vx=0,vy=0,vz=0)
            
            return TIMEOUT
>>>>>>> 5a18978 (Third day testing changes)

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"PRECISION_LANDING Failed: {e}")
            return ABORT

