# File that will have all the constants we will use in the task
SIM_MODE = True

CAMERA_SOURCE = '/down_camera'
PHOTOS_FOLDER = '' #Folder where we will save the images for the dataset
FRAMES_FOLDER = ''
#Folder to save the frames that we will process with Yolo

IMAGE_WIDTH = 640 #NOTE: Need to be certain about this ones
IMAGE_HEIGHT = 640

TAKEOFF_HEIGHT = 6 #Meters, can change this later
MAX_ALTITUDE = 7 #meters

SEARCH_TIME = 120 #seconds (2min)
FIND_TIME = 60 #seconds (1min)

MARKER_DICT = 5 #ArUco of 5x5
ARUCO_SIZE = 0.25 #ArUco size
WAYPOINTS = [ (3, 3), (3, -3), (-3, -3), (-3, 3)]