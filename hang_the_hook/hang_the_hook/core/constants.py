from nectar.control import PIDController, PIDConfig

SIM_MODE = True

'''
Flying config
'''
RTL_ALTITUDE = 5.0
TAKEOFF_HEIGHT = 5.0

'''
Image Handler constants
'''
IMAGE_WIDTH = 640
IMAGE_HEIGHT = 480
SIM_IMAGE_COMPRESSED = False

'''
"webcam" -> in-built camera of personal machine
"/down_camera" -> mavros topic for SIM drone camera
'''
IMAGE_SOURCE = "/down_camera"

'''
Drone frame
'''
FRAME_WIDTH = 1280
FRAME_HEIGHT = 980

'''
PID constants
'''
# PID constants - Eixo X
PID_X_KP = 0.5
PID_X_KI = 0.0
PID_X_KD = 0.0001
PID_X_OUTPUT_LIMITS = (-0.7, 0.7)
PID_X_INTEGRAL_LIMITS = (-0.5, 0.5)

# PID constants - Eixo Y
PID_Y_KP = 0.1
PID_Y_KI = 0.0
PID_Y_KD = 0.0001
PID_Y_OUTPUT_LIMITS = (-0.7, 0.7)
PID_Y_INTEGRAL_LIMITS = (-0.5, 0.5)

# PID constants - Yaw / Ângulo
PID_YAW_KP = 0.5
PID_YAW_KI = 0.0
PID_YAW_KD = 0.001
PID_YAW_OUTPUT_LIMITS = (-0.65, 0.65)
PID_YAW_INTEGRAL_LIMITS = (-0.05, 0.05)
