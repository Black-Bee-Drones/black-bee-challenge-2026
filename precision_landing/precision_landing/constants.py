# File that will have all the constants we will use in the task

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

SIM_MODE = True

CAMERA_SOURCE = "/down_camera" #We will use "webcam" for the drone

DETECTOR_MODEL_SOURCE = str(Path(get_package_share_directory("precision_landing")) / "models" / "best_detector1.pt")

DETECTOR_CONFIDENCE_THRESHOLD = 0.3

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 640

TAKEOFF_HEIGHT = 5 #Meters
MAX_ALTITUDE = 6 #meters

SEARCH_TIME = 160 #seconds (2min 40s)
<<<<<<< HEAD
<<<<<<< HEAD
FIND_TIME = 120 #seconds (2min)
<<<<<<< HEAD
PRECISION_LANDING_TIME = 600
FIND_BUFFER = []

MARKER_DICT = 5 #ArUco of 5x5
ARUCO_SIZE = 0.25 #ArUco size
WAYPOINTS = [(0, 0),  (0,3), (3, 0), (0, -3), (3, -3)]
=======
=======
FIND_TIME = 160 #seconds (2min)
>>>>>>> 103defb (feat: telo obedeceu pedro aguas belas e commitou)
=======
FIND_TIME = 160 #seconds (2min 40s)
>>>>>>> da5bf1c (Yolo neles)
PRECISION_LANDING_TIME = 120

MARKER_DICT = 5 #ArUco of 5x5
ARUCO_SIZE = 0.25 #ArUco size
<<<<<<< HEAD
WAYPOINTS = [(0,0), (-4,5), (-1,5), (3,5), (5,5), (5,3), (3,3), (0,3),
(-2,3), (-4,3), (-4,1), (-2,1), (2,1), (5,1), (5,-2), (2,-2), (-1,-2), (-4,-2),
<<<<<<< HEAD
(-4,-4), (-2,-4), (1,-4), (3,-4), (5, -4)]
>>>>>>> 5a18978 (Third day testing changes)
=======
=======
WAYPOINTS = [(0,0), (-4,5), (-1,5), (1,5), (3,5), (5,5), (5,3), (3,3), (0,3),
(-2,3), (-4,3), (-4,1), (-2,1), (0,1), (2,1), (5,1), (5,-2), (2,-2), (-1,-2), (-4,-2),
>>>>>>> 3aa4d87 (I milagroso e waypoint q n sei)
(-4,-4), (-2,-4), (1,-4), (4,-4)]
>>>>>>> 103defb (feat: telo obedeceu pedro aguas belas e commitou)

<<<<<<< HEAD
CONTROLER_P_XY = 0.350 if SIM_MODE else 0.123
CONTROLER_I_XY = 0.103
CONTROLER_D_XY = 0.05
=======
CONTROLER_P_XY = 0.250 if SIM_MODE else 0.123
CONTROLER_I_XY = 0.0012
CONTROLER_D_XY = 0.0
>>>>>>> 3aa4d87 (I milagroso e waypoint q n sei)
CONTROLER_OUTPUT_LIMITS_XY = (-0.44, 0.44)
CONTROLER_INTEGRAL_LIMITS_XY = (-0.10, 0.10)
PRECISE_DOWN_TOLERANCE_PX = 100

CONTROLER_P_Z = 0.20
CONTROLER_I_Z = 0.0
CONTROLER_D_Z = 0.0
CONTROLER_OUTPUT_LIMITS_Z = (-0.8, 0.8)
CONTROLER_INTEGRAL_LIMITS_Z = (-0.1, 0.1)

FINAL_LANDING_TOLERANCE = 0.2
FINAL_LANDING_HEIGHT = 1.2