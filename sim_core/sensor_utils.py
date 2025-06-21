# # sim_core/sensor_utils.py
# """
# Simulates sensor fusion and ranging utilities.
# """
# import random
# import math
# from typing import Tuple

# # Store previous fused position for complementary filter
# _fuse_prev = (0.0, 0.0, 0.0)

# def get_corrected_position(raw_pos: Tuple[float, float, float], source: str = 'GPS') -> Tuple[float, float, float]:
#     """
#     Apply positional noise based on source.
#     - 'GPS': ~2-5m horizontal, ~5-10m vertical error
#     - 'RTK': ~0.03m horizontal/vertical error
#     """
#     x, y, z = raw_pos
#     if source == 'RTK':
#         noise_x = random.gauss(0, 0.03)
#         noise_y = random.gauss(0, 0.03)
#         noise_z = random.gauss(0, 0.03)
#     else:
#         noise_x = random.gauss(0, 2.0)
#         noise_y = random.gauss(0, 2.0)
#         noise_z = random.gauss(0, 5.0)
#     return (x + noise_x, y + noise_y, z + noise_z)


# def fuse_gps_imu(gps_pos: Tuple[float, float, float], imu_delta: Tuple[float, float, float], alpha: float = 0.98) -> Tuple[float, float, float]:
#     """
#     Complementary filter fusing GPS (low rate, noisy) with IMU dead-reckoning (high rate, drifting).
#     new = alpha*(prev + imu_delta) + (1-alpha)*gps_pos
#     """
#     global _fuse_prev
#     x_gps, y_gps, z_gps = gps_pos
#     dx, dy, dz = imu_delta
#     x_prev, y_prev, z_prev = _fuse_prev
#     # Dead-reckon from previous fused
#     x_dr, y_dr, z_dr = x_prev + dx, y_prev + dy, z_prev + dz
#     # Fuse
#     x = alpha * x_dr + (1 - alpha) * x_gps
#     y = alpha * y_dr + (1 - alpha) * y_gps
#     z = alpha * z_dr + (1 - alpha) * z_gps
#     _fuse_prev = (x, y, z)
#     return (x, y, z)


# def get_range(pos_i: Tuple[float, float, float], pos_j: Tuple[float, float, float], sigma: float = 0.1) -> float:
#     """
#     Simulate peer-to-peer UWB ranging with Gaussian noise (~10cm).
#     """
#     true_d = math.dist(pos_i, pos_j)
#     return true_d + random.gauss(0, sigma)

# Gemini

# sim_core/sensor_utils.py
"""
Simulates sensor utilities, now with `fuse_gps_imu` designed for external state management.
"""
import random
import math
from typing import Tuple

def get_corrected_position(raw_pos: Tuple[float, float, float], source: str = 'GPS') -> Tuple[float, float, float]:
    """
    Apply positional noise based on source.
    - 'GPS': ~2-5m horizontal, ~5-10m vertical error
    - 'RTK': ~0.03m horizontal/vertical error
    """
    x, y, z = raw_pos
    if source == 'RTK':
        noise_x = random.gauss(0, 0.03)
        noise_y = random.gauss(0, 0.03)
        noise_z = random.gauss(0, 0.03)
    else:
        noise_x = random.gauss(0, 2.0)
        noise_y = random.gauss(0, 2.0)
        noise_z = random.gauss(0, 5.0)
    return (x + noise_x, y + noise_y, z + noise_z)


def fuse_gps_imu(gps_pos: Tuple[float, float, float], imu_delta: Tuple[float, float, float], prev_fused_pos: Tuple[float, float, float], alpha: float = 0.98) -> Tuple[float, float, float]:
    """
    Complementary filter fusing GPS (low rate, noisy) with IMU dead-reckoning (high rate, drifting).
    It now accepts `prev_fused_pos` as an argument, making it stateless internally.
    new = alpha*(prev + imu_delta) + (1-alpha)*gps_pos
    """
    x_gps, y_gps, z_gps = gps_pos
    dx, dy, dz = imu_delta
    x_prev, y_prev, z_prev = prev_fused_pos

    # Dead-reckon from previous fused position
    x_dr, y_dr, z_dr = x_prev + dx, y_prev + dy, z_prev + dz

    # Fuse with GPS
    x = alpha * x_dr + (1 - alpha) * x_gps
    y = alpha * y_dr + (1 - alpha) * y_gps
    z = alpha * z_dr + (1 - alpha) * z_gps

    return (x, y, z)


def get_range(pos_i: Tuple[float, float, float], pos_j: Tuple[float, float, float], sigma: float = 0.1) -> float:
    """
    Simulate peer-to-peer UWB ranging with Gaussian noise (~10cm).
    """
    true_d = math.dist(pos_i, pos_j)
    return true_d + random.gauss(0, sigma)

