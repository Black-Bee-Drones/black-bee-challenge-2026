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
# PID gains
KP = 0.01
KI = 0.0
KD = 0.002
