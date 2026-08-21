from dataclasses import dataclass, field


@dataclass
class Config:
    target_box: tuple = (2, 2)

    # Simulation
    sim_mode: bool = True
    sim_image_source: str = "/down_camera"
    sim_image_compressed: bool = False

    # Camera
    image_source : str = 'webcam'   # 'ros' for Simulation Mode 
    image_width: int = 640          # pixels
    image_height: int = 480         # pixels

    # Approach
    approach_timeout: int = 60         # seconds
    approach_tolerance: float = 0.20   # meters
    dropoff_altitude: float = 0.80     # meters
    dropoff_tolerance: float = 0.10    # meters

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
    
    # waypoints: boxes
    delivery_boxes: list = field(default_factory=list)
    
    # Gripper Controller
    has_thePkg : bool = True   # flag to verify if the drone has the package (True)              
    servo_channel : int = 0           # aux_out (0-7 maps to AUX physical outputs 1-8)
    servo_open_pwm : int = 1800
    servo_closed_pwm : int = 1200     
    servo_action_delay = 1.0    # sleep time
    

    # PIDController ###
    # PID xy
    xy_output_lim: tuple = (-1.0, 1.0)
    xy_integral_lim: tuple = (-1.0, 1.0)
    x_kp: float = 0.5
    x_ki: float = 0.1
    x_kd: float = 0.0
    y_kp: float = 0.5
    y_ki: float = 0.1
    y_kd: float = 0.0

    # PID z
    z_kp: float = 1.0
    z_kd: float = 1.0
    z_ki: float = 1.0
    z_output_lim: tuple = (-1.0, 1.0)
    z_integral_lim: tuple = (-1.0, 1.0)

    # PID yaw
    controller_yaw_kp: float = 1.0
    controller_yaw_kd: float = 1.0
    controller_yaw_ki: float = 0
    controller_yaw_output_min: float = -1.0
    controller_yaw_output_max: float = 1.0
    controller_yaw_integral_min: float = -1.0
    controller_yaw_integral_max: float = 1.0
    