from dataclasses import dataclass


@dataclass(frozen=True)
class Config:
    
    #Drone
    drone_type : str = 'mavlink'
    conection_string: str = 'TODO: add the protocol, the address, and the port'
    
    
    #SIMULATION
    SIM_MODE : bool = True
    


@dataclass(frozen=True)
class SITLConfig(Config):
    conection_string: str = 'tcp:127.0.0.1:5762'