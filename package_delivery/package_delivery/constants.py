import os
from ament_index_python.packages import get_package_share_directory
from dataclasses import dataclass, field

@dataclass
class Config:
    # target_box: tuple = (
    #     (0, 0),     # box 1: lat, long
    #     (1, 1),     # box 2: lat, long
    #     (2, 2),     # box 3: lat, long
    # )
    
    target_box: tuple = (
        (-22.4153391, -45.4479390),     # box 1: lat, long
        (-22.4153820, -45.4479658),     # box 2: lat, long
        (2, 2),     # box 3: lat, long
    )

    # Simulation
    sim_mode: bool = False
    sim_image_source: str = "/down_camera"
    sim_image_compressed: bool = False
    sim_target_box: tuple = (
        (-35.363292, 149.165253),   # box 1: lat, long
        (-35.363284, 149.165271),   # box 2: lat, long
        (-35.363291, 149.165307),   # box 3: lat, long
    )

    # Model - v3 box - 
    box_model_source: str = os.path.join(get_package_share_directory('package_delivery'), 'models', 'best.pt')
    box_class_name: str = "box"
    box_conf: float = 0.45      # before: 0.25

    # Camera
    image_source : str = 'webcam'
    image_width: int = 640          # pixels
    image_height: int = 480         # pixels

    # Approach
    approach_timeout: int = 80         # seconds
    approach_tolerance: float = 0.20   # meters
    approach_tolerance_px: int = 100   # pixels
    dropoff_altitude: float = 0.80     # meters
    dropoff_tolerance: float = 0.20    # meters
    required_frames: int = 3
    lost_tolerance: int = 13

    altitude_inc: float = 0.8 # meters
    safe_altitude: float = 3 # meters
    max_altitude: float = 5.0  # meters
    
    drone_type: str = 'mavlink'
    #connection_string: str = 'udp:127.0.0.1:14551' #STIL
    connection_string: str = '/dev/ttyAMA1' # '/dev/ttyAMA1' # "/dev/ttyTHS1" ou /dev/ttyUSB0, dependendo de como conectou (baud=921600 for tests)
    
    # Takeoff and Land
    takeoff_altitude: float = safe_altitude
    rtl_altitude: float = 1.5   # meters
    
    # Center
    center_threshold_xy: float = 0.2  # meters
    center_threshold_z: float = 0.2  # meters
    center_threshold_yaw: float = 5.0  # degrees
    land_altitude: float = 1.0  # meters
    
    # Gripper Controller
    has_thePkg : bool = True        # flag to verify if the drone has the package (True)              
    servo_channel : int = 7         # aux_out (0-7 maps to AUX physical outputs 1-8)
    servo_open_pwm : int = 1000     # padeiro deu os valores
    servo_closed_pwm : int = 1800   # padeiro deu os valores   
    servo_action_delay = 3.0    # sleep time


    # PIDController ###
    # PID xy
    xy_output_lim: tuple = (-1.0, 1.0)
    xy_integral_lim: tuple = (-1.0, 1.0)   
    
    x_kp: float = 0.123
    x_ki: float = 0.0
    x_kd: float = 0.02    
    
    y_kp: float = 0.123
    y_ki: float = 0.0
    y_kd: float = 0.02

    # PID z
    z_kp: float = 0.45        
    z_ki: float = 0.0
    z_kd: float = 0.0                       # zero — é o eixo mais sensível a overshot
    z_output_lim: tuple = (-0.3, 0.15)      # desce mais devagar que sobe
    z_integral_lim: tuple = (-1.0, 1.0)

    # PID yaw
    # controller_yaw_kp: float = 1.0
    # controller_yaw_kd: float = 1.0
    # controller_yaw_ki: float = 0
    # controller_yaw_output_min: float = -1.0
    # controller_yaw_output_max: float = 1.0
    # controller_yaw_integral_min: float = -1.0
    # controller_yaw_integral_max: float = 1.0
    