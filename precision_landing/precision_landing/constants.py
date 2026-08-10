# File that will have all the constants we will use in the task

from pathlib import Path
from ament_index_python.packages import get_package_share_directory

SIM_MODE = True

CAMERA_SOURCE = '/down_camera'
FRAMES_FOLDER = ''
#Folder to save the frames that we will process with Yolo
DETECTOR_MODEL_SOURCE = str(Path(get_package_share_directory("precision_landing")) / "models" / "best_detector.pt")
#TODO: update this later to get our best detector model
DETECTOR_CONFIDENCE_THRESHOLD = 0.6

IMAGE_WIDTH = 640
IMAGE_HEIGHT = 640

TAKEOFF_HEIGHT = 6 #Meters, can change this later
MAX_ALTITUDE = 7 #meters

SEARCH_TIME = 120 #seconds (2min)
FIND_TIME = 120 #seconds (2min)

MARKER_DICT = 5 #ArUco of 5x5
ARUCO_SIZE = 0.25 #ArUco size
WAYPOINTS = [ (1, 1), (2, 2), (3, 3), (3, 2), (3, 1), (3, 0),
             (3, -1), (3, -2), (3, -3), (2, -3), (1, -3),
             (0, -3), (-1, -3), (-2, -3) (-3, -3), 
             (-3, -2), (-3, -1), (-3, 0), (-3, 1), (-3, 2) (-3, 3)] #Need to update these for the arena size of 14x14