from dataclasses import dataclass
# Simulation
SIM_MODE = 1
SIM_IMAGE_SOURCE = "/down_camera"
SIM_IMAGE_COMPRESSED = False

# Camera
IMAGE_WIDTH = 0     # Change later
IMAGE_HEIGHT = 0    # Change later


@dataclass
class Config:
    
    safe_altitude: float = 2.0 # meters
    max_altitude: float = 6.0  # meters
    
    drone_type: str = 'mavlink'
    connection_string: str = 'udp:127.0.0.1:14551'
    
    # Takeoff and Land
    takeoff_altitude: float = safe_altitude
    rtl_altitude: float = 1.2   # meters
    
    # Center
    center_threshold_xy: float = 0.2  # meters
    center_threshold_z: float = 0.2  # meters
    center_threshold_yaw: float = 5.0  # degrees
    lost_tolerance: int = 5
    land_altitude: float = 1.0  # meters
    
    # waypoints: launch and package bases; boxes
    launch_bases: list[dict] = []
    pkgs_bases: list[dict] = []
    delivery_boxes: list[dict] = []
    
    # Gripper Controller
    has_thePkg : bool = True   # flag to verify if the drone has the package (True)              
    servo_channel : int = 0           # aux_out (0-7 maps to AUX physical outputs 1-8)
    servo_open_pwm : int = 1800
    servo_closed_pwm : int = 1200     
    servo_action_delay = 1.0    # sleep time
    

    ### PIDController ###
    # PID xy
    controller_xy_kp: float = 1.0
    controller_xy_kd: float = 1.0
    controller_xy_ki: float = 1.0
    controller_xy_output_min: float = -1.0
    controller_xy_output_max: float = 1.0
    controller_xy_integral_min: float = -1.0
    controller_xy_integral_max: float = 1.0

    # PID z
    controller_z_kp: float = 1.0
    controller_z_kd: float = 1.0
    controller_z_ki: float = 1.0
    controller_z_output_min: float = -1.0
    controller_z_output_max: float = 1.0
    controller_z_integral_min: float = -1.0
    controller_z_integral_max: float = 1.0

    # PID yaw
    controller_yaw_kp: float = 1.0
    controller_yaw_kd: float = 1.0
    controller_yaw_ki: float = 0
    controller_yaw_output_min: float = -1.0
    controller_yaw_output_max: float = 1.0
    controller_yaw_integral_min: float = -1.0
    controller_yaw_integral_max: float = 1.0