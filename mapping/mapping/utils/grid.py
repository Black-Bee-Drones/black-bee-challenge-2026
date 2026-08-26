import math 
from typing import List, Dict, Tuple, Optional
from enum import Enum

from mapping.config import Config

class PatternType(Enum):
    COLUMNS = "COLUMNS"
    ROWS = "ROWS"
    
class Direction(Enum):
    FOWARD = "FOWARD"
    BACKWARD = "BAKWARD"
    RIGHT = "RIGHT"
    LEFT = "LEFT"
    
class Grid:
    def __init__(self, config : Config):
        
        self.config = config
        self.search_width = self.config.arena.size_x_m
        self.search_height = self.config.arena.size_y_m
        self.grid_spacing
        self.initial_position : Tuple[float, float] = (0.0, 0.0)
        self.start_offset : Tuple [float, float] = (0.0,  0.0)
        self.pattern_type = PatternType("COLUMNS"),
        self.primary_direction = Direction("FORWARD"),
        self.transition_direction = Direction("RIGHT"),
        
        
        self.search_origin = (
            self.initial_position[0] + self.start_offset[1],
            self.initial_position[1] + self.start_offset[1],
        )
        
        
        def _generate_boustrophedon_pattern(self) -> List[Dict[str, any]]:
            waypoints = []
            
            if self.pattern_type == PatternType.COLUMNS:
                waypoints = self.generate_column_pattern()
            else:
                waypoints = self.generate_row_pattern()
            
            return waypoints
                
        
        
        def _generate_column_pattern(self) -> List[Dict[str, any]]:
            
            waypoints = []
            
            num_columns = max(1, int(math.ceil(self.search_width / self.grid_spacing))) + 1

            points_per_column = max(
                1, int(math.ceil(self.search_height / self.grid_spacing))
            )
        
            print(f"Num Columns:{num_columns}, Points: {points_per_column}")
            
            for col in range(num_columns):
                if self.transition_direction == Direction.RIGHT:

                    y_pos = self.search_origin[1] - col * self.grid_spacing
                else:  # LEFT

                    y_pos = self.search_origin[1] + col * self.grid_spacing
         
            
             