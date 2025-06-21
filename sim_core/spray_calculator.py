# sim_core/spray_calculator.py
"""
Calculates spray volume, timing, and compensation for wind and material properties.
"""
from typing import Tuple
import math
def compute_spray_duration(flow_rate: float, volume: float) -> float:
    """Return seconds to spray given volume at flow_rate (units: m^3/s)"""
    return volume / flow_rate

def wind_compensation(wind_speed: float, wind_direction: float,
                      spray_direction: float, base_angle: float = 45.0) -> float:
    """Adjust nozzle angle to compensate for crosswind (degrees)"""
    # difference between wind vector and spray
    delta = abs(wind_direction - spray_direction)
    return base_angle + wind_speed * math.cos(math.radians(delta))