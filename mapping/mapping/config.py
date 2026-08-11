import yaml
from dataclasses import dataclass
from enum import Enum

class LandingMode(str, Enum):
    LAND = 'LAND'
    RTL = 'RTL'

@dataclass(frozen=True)
class Config:
    
    drone_type : str = "mavlink"
    
    conection_string : str = 'tcp:127.0.0.1:5760'
    
    sim_mode: bool = True
    
    takeoff_altitude: float = 1.6 #meters
    
    landing_mode: LandingMode = LandingMode.LAND

    down_image_source : str = "ros"
    down_ros_topic: str = "/down_camera"
    #camera_id: int
    
    image_width : int = 640
    image_height : int = 640

    # @classmethod
    # def load(cls, filepath="config.yml"):
    #     with open(filepath, "r") as f:
    #         data = yaml.safe_load(f)

    #     return cls(
    #         drone_type=data["drone"]["type"],
    #         conection_string=data["drone"]["connection_string"],
    #         sim_mode=data["simulation"]["mode"],
    #         takeoff_altitude=data["takeoff"]["altitude"],
    #         landing_mode=LandingMode(data["land"]["mode"]),
    #         down_image_source=data["camera"]["down_source"],
    #         image_width=data["image"]["width"],
    #         image_height=data["image"]["height"]
    #     )

# Para usar no código:
# config = Config.load()


@dataclass(frozen=True)
class SITLConfig(Config):
    connection_string: str = 'tcp:127.0.0.1:5760'

    front_image_source: str = 'ros'
    front_ros_topic: str = '/front_camera/image'
    down_image_source: str = 'ros'
    down_ros_topic: str = '/down_camera'