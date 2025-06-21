# sim_core/physics_utils.py
"""
Optional physics utilities for drag, decay, and environmental effects.
"""
import math
from typing import Tuple

def drag_force(velocity: float, drag_coefficient: float, area: float, density: float = 1.225) -> float:
    """Compute drag force (F = 0.5 * rho * v^2 * Cd * A)"""
    return 0.5 * density * velocity**2 * drag_coefficient * area

def decay_concentration(initial: float, rate: float, time_s: float) -> float:
    """Exponential decay C = C0 * e^{-rate * t}"""
    return initial * math.exp(-rate * time_s)
