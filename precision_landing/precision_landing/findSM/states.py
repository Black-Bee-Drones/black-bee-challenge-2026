from nectar.vision import(
    ImageHandler,
    Aruco,
)
from nectar.control import(
    MavrosDrone,
    MoveReference,
)

from rclpy.duration import Duration

import yasmin
from yasmin import State, Blackboard
from yasmin_ros.basic_outcomes import SUCCEED, FAIL, TIMEOUT, ABORT
from yasmin_ros.yasmin_node import YasminNode

from precision_landing.constants import(
    SEARCH_TIME,
    FIND_TIME,
    MAX_ALTITUDE,
    MARKER_DICT,
    ARUCO_SIZE,
    WAYPOINTS,
    FIND_BUFFER,
)

T_P_F = False #TARGET PRE FOUND

class Search(State): #Sub-state that will only move around the arena until it detects an ArUco
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, TIMEOUT])
        self.node = YasminNode.get_instance()

    def execute(self, blackboard: Blackboard):
        try:
            global T_P_F

            if "drone" not in blackboard: #IFs que conferem se está tudo certo antes de iniciar o estado
                yasmin.YASMIN_LOG_ERROR("Drone not Available... ")
                return ABORT
            drone: MavrosDrone = blackboard["drone"]
                        
            if "camera" not in blackboard:
                yasmin.YASMIN_LOG_ERROR("Camera not Available... ")
                return ABORT
            camera: ImageHandler = blackboard["camera"]
            
            if not camera:
                yasmin.YASMIN_LOG_ERROR("Camera or ImageHandler not initialized")
                return ABORT
            drone.delay(0.5)

            aruco = Aruco(marker_dict=MARKER_DICT, tag_size=ARUCO_SIZE)

            start_time = self.node.get_clock().now() #gets the start time of the state
            search_time = Duration(seconds=SEARCH_TIME) #gets the max time in seconds before TIMEOUT

            idx = 0

            while (self.node.get_clock().now() - start_time) < search_time: #Executes this sub-state for a max of 2min
                if drone.get_altitude() >= MAX_ALTITUDE: #Aborts if the drone gets past 6m of altitude
                    yasmin.YASMIN_LOG_ERROR('Failed: limit altitude reached.')
                    drone.move_velocity(0.0, 0.0, 0.0, 0.0)
                    drone.delay(1.0)        
                    return ABORT

                for _ in range(2): #Repeats the detection 2 times for security
                    frame = camera.take_photo()
                    bbox, aruco_id = aruco.detect(frame.image, draw=True)
                    
                    if aruco_id is not None:
                        blackboard["aruco_id"] = str(aruco_id) #The index for the numbers are 0, 1 and 2
                        drone.move_velocity(vx=0.0, vy=0.0, vz=0.0)
                        yasmin.YASMIN_LOG_INFO("Detected the ArUco, moving closer... ")
                        yasmin.YASMIN_LOG_INFO(f"ARUCO ID: {aruco_id}")

                        yaw_angle = aruco.calculateYawFromCorners(bbox=bbox)
<<<<<<< HEAD
                    
                        try:
                            aruco_shape = self.get_aruco_shape(frame, bbox)
                        except Exception as i:
                            yasmin.YASMIN_LOG_INFO("ERRO AO DETECTAR ARUCO")
                            yasmin.YASMIN_LOG_INFO(f"{i}")
                            drone.move_to(yaw=yaw_angle)
                            drone.move_to(x=1.0, reference=MoveReference.BODY)

=======

                        try:
                            aruco_shape = self.get_aruco_shape(frame, bbox)
                        except Exception as i:
                            yasmin.YASMIN_LOG_INFO(f"DIDN'T DETECT shape..., retaking photo")
                            yasmin.YASMIN_LOG_INFO(f"{i}")
                            drone.move_to(yaw=yaw_angle)
                            drone.move_to(x=1, reference=MoveReference.BODY)
<<<<<<< HEAD
>>>>>>> b3aba2b (Arrumando bobeira da imagem cortada)
=======
                            
>>>>>>> 53c9f01 (Esqueci de voltar o yaw)
                            frame = camera.take_photo()
                            bbox2, id = aruco.detect(frame.image)

                            aruco_shape = self.get_aruco_shape(frame, bbox2)
<<<<<<< HEAD
<<<<<<< HEAD
                            drone.move_to(yaw=-yaw_angle)
=======

>>>>>>> b3aba2b (Arrumando bobeira da imagem cortada)
=======
                        
                        drone.move_to(yaw=-yaw_angle)
>>>>>>> 53c9f01 (Esqueci de voltar o yaw)
                        blackboard["aruco_shape"] = aruco_shape
                        yasmin.YASMIN_LOG_INFO(f"Aruco shape detected: {aruco_shape}")

                        #sees if it's in the buffer, if so it goes instantly
                        search_key = f"{aruco_shape}{aruco_id}"
                        yasmin.YASMIN_LOG_INFO(f"search_key: {search_key}")

                        for item in FIND_BUFFER:
                            if item.startswith(search_key):
                                palavra, position = item.split(":", 1)

                                position = position.strip("()")
                                x, y = position.split(",")

                                x = float(x)
                                y = float(y)
                                T_P_F = True
                                yasmin.YASMIN_LOG_INFO("TARGET ALREADY PREFOUND, GOING TO HIM...")
                                drone.move_to(x=x,y=y, reference=MoveReference.TAKEOFF)
                                break
                        return SUCCEED

                if idx < len(WAYPOINTS):
                    x, y = WAYPOINTS[idx]
                    drone.move_to(x=x, y=y, z=0, reference=MoveReference.TAKEOFF)
                    self.ADD_FINDING_BUFFER(camera=camera, waypoint=WAYPOINTS[idx], drone = drone)
                    idx += 1
                    drone.delay(0.5)
                else:
                    yasmin.YASMIN_LOG_INFO("FAILED, didn't find the ArUco")
                    return FAIL

            return TIMEOUT

        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"SEARCH SUB-STATE FAILED: {e}")
            return ABORT

    def get_aruco_shape(self, frame, bbox): #Function that gets the shape around the ArUco
        aruco_shape = None
        for s in frame.filter_by_class(['Triangle', 'Hexagon', 'Star']):
            if(abs(self.bbox_center(bbox)[0]-s.center[0])<=s.width/2):
                aruco_shape = s.class_name

        if aruco_shape is not None:
            return str(aruco_shape)
        else:
            return None

    def ADD_FINDING_BUFFER(self, camera, waypoint, drone):
        pre_buffer = []

        #analisa detecções na imagem para ter certeza que está certo oque está detectando
        for _ in range(5):
            yasmin.YASMIN_LOG_INFO("FAZENDO CAPTURAS...")
            result = camera.take_photo()
            for s in result.filter_by_class(['Triangle', 'Hexagon', 'Star']):
                for n in result.filter_by_class(['3', '4', '5']):
                    if(abs(s.center[0]-n.center[0])<=s.width/2 and abs(s.center[1] - n.center[1]) <= s.height/2):
                        pre_buffer.append(f"{s.class_name + n.class_name}:{waypoint}")
                        yasmin.YASMIN_LOG_INFO(f"DETECTANDO A CLASSE: {s.class_name}")
            drone.delay(0.1)

        #faz a limpa dos bufferes para ver se esta correto
        for item in pre_buffer:
            if pre_buffer.count(item) >= 4 and item not in FIND_BUFFER:
                FIND_BUFFER.append(item)
                while item in pre_buffer:
                    pre_buffer.remove(item)
        yasmin.YASMIN_LOG_INFO(f"FIND_BUFFER: {FIND_BUFFER}")                

                        

    def bbox_center(self, bbox): #Function to get the center of the ArUco by its bbox
        corners = bbox[0][0]  # unwrap: tupla -> array (1,4,2) -> array (4,2)
    
        sup_left = corners[0]
        inf_right = corners[2]

        cx = (sup_left[0] + inf_right[0]) / 2
        cy = (sup_left[1] + inf_right[1]) / 2
        center = (cx, cy)
        return center



class FindTargetBase(State):
    def __init__(self):
        super().__init__(outcomes=[SUCCEED, ABORT, FAIL, TIMEOUT])
        self.node = YasminNode.get_instance()
    
    def execute(self, blackboard: Blackboard):
        try:

            global T_P_F
            #check if target was pre found
            if T_P_F:
                yasmin.YASMIN_LOG_INFO("TARGET ALREADY PREFOUD...")
                return SUCCEED
            
            drone: MavrosDrone = blackboard["drone"]
                    
            camera: ImageHandler = blackboard["camera"]
                                        
            if not camera:
                yasmin.YASMIN_LOG_ERROR("Camera or ImageHandler not initialized")
                return ABORT

            start_time = self.node.get_clock().now() #gets the start time of the state
            find_time = Duration(seconds=FIND_TIME) #gets the max time in seconds before TIMEOUT

            idx = 0

            drone.move_to(x=0.0, y=0.0, reference=MoveReference.TAKEOFF)
            
            while (self.node.get_clock().now() - start_time) < find_time:
                    
                if drone.get_altitude() >= MAX_ALTITUDE: #Aborts if the drone gets past 6m of altitude
                    yasmin.YASMIN_LOG_ERROR('Failed: limit altitude reached.')
                    drone.move_velocity(0.0, 0.0, 0.0, 0.0)
                    drone.delay(1.0)
                    return ABORT
                    
                for _ in range(2):
                    frame = camera.take_photo()

                    for s in frame.filter_by_class([blackboard["aruco_shape"]]):
                        for n in frame.filter_by_class([blackboard["aruco_id"]]):
                            if (abs(n.center[0] - s.center[0]) <= s.width/2) and (abs(n.center[1] - s.center[1]) <= s.height/2):
                                #Verify if there's a base with the aruco_id inside the aruco_shape we want
                                yasmin.YASMIN_LOG_INFO("DETECTED TARGET BASE")
                                return SUCCEED

                if idx < len(WAYPOINTS):
                    x, y = WAYPOINTS[idx]
                    drone.move_to(x=x, y=y, z=0, reference=MoveReference.TAKEOFF)
                    idx += 1
                    drone.delay(0.5)
                else:
                    yasmin.YASMIN_LOG_INFO("FAILED, didn't find the equivalent base")
                    return FAIL
                    
            return TIMEOUT
        
        except Exception as e:
            yasmin.YASMIN_LOG_ERROR(f"GET_TARGET_BASE SUB-STATE FAILED: {e}")
            return ABORT