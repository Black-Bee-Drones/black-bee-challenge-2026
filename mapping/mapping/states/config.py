from dataclasses import dataclass
from enum import Enum


class LandingMode(str, Enum):
    LAND = 'LAND',
    RTL = 'RTL'



@dataclass(frozen=True)
class Config:
    
    #Drone
    drone_type : str = 'mavlink'
    conection_string: str = 'TODO: add the protocol, the address, and the port'
    
    
    #SIMULATION
    SIM_MODE : bool = True
    
    
    ###TAKEOFF###
    takeoff_altitude = 5.6 #meters
    
    
    ###LAND###
    landing_mode : LandingMode = LandingMode.LAND


@dataclass(frozen=True)
class SITLConfig(Config):
    conection_string: str = 'tcp:127.0.0.1:5762'