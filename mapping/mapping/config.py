import yaml
from dataclasses import dataclass
from enum import Enum

class LandingMode(str, Enum):
    LAND = 'LAND'
    RTL = 'RTL'

@dataclass(frozen=True)
class Config:
    drone_type: str
    conection_string: str
    sim_mode: bool
    takeoff_altitude: float
    landing_mode: LandingMode

    @classmethod
    def load(cls, filepath="config.yml"):
        with open(filepath, "r") as f:
            data = yaml.safe_load(f)

        return cls(
            drone_type=data["drone"]["type"],
            conection_string=data["drone"]["connection_string"],
            sim_mode=data["simulation"]["mode"],
            takeoff_altitude=data["takeoff"]["altitude"],
            landing_mode=LandingMode(data["land"]["mode"])
        )

# Para usar no código:
# config = Config.load()
