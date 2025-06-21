# # sim_core/simulation.py
# """
# Provides a simple simulation/test harness for the swarm core modules.
# Run this in test mode before live deployment.
# """
# import time
# import random
# from typing import List, Tuple
# from sensor_utils import get_corrected_position, fuse_gps_imu, get_range
# from flocking_controller import FlockingController
# from formation_planner import FormationPlanner

# # Simulation parameters
# NUM_DRONES = 10
# TIMESTEPS = 200
# DT = 0.1  # seconds per step

# # Initial random positions and zero velocities
# raw_positions: List[Tuple[float, float, float]] = [
#     (random.uniform(-5,5), random.uniform(-5,5), 10.0) for _ in range(NUM_DRONES)
# ]
# imu_deltas: List[Tuple[float, float, float]] = [(0.0,0.0,0.0)] * NUM_DRONES
# velocities: List[Tuple[float, float, float]] = [(0.0,0.0,0.0)] * NUM_DRONES

# # Generate a wall formation covering 3m target + 2m buffer
# targets = FormationPlanner.wall(NUM_DRONES, spacing=5.0, altitude=10.0)

# # Create controller
# flock = FlockingController(
#     perception_radius=6.0,
#     max_speed=3.0,
#     max_force=0.1,
#     nominal_spacing=5.0
# )

# # Run simulation
# for step in range(TIMESTEPS):
#     # Simulate IMU deltas (here zero or small noise)
#     imu_deltas = [(
#         random.gauss(0,0.01), random.gauss(0,0.01), random.gauss(0,0.005)
#     ) for _ in range(NUM_DRONES)]
#     # Compute accelerations
#     accs = flock.compute(
#         raw_positions,
#         imu_deltas,
#         velocities,
#         targets,
#         use_fusion=True
#     )
#     # Update velocities and positions
#     new_vel = []
#     new_pos = []
#     for i in range(NUM_DRONES):
#         vx, vy, vz = velocities[i]
#         ax, ay, az = accs[i]
#         # Euler integration
#         vx_new = vx + ax * DT
#         vy_new = vy + ay * DT
#         vz_new = vz + az * DT
#         # limit speed
#         speed = (vx_new**2 + vy_new**2 + vz_new**2)**0.5
#         if speed > flock.max_speed:
#             factor = flock.max_speed / speed
#             vx_new *= factor; vy_new *= factor; vz_new *= factor
#         x, y, z = raw_positions[i]
#         x_new = x + vx_new * DT
#         y_new = y + vy_new * DT
#         z_new = z + vz_new * DT
#         new_vel.append((vx_new, vy_new, vz_new))
#         new_pos.append((x_new, y_new, z_new))
#     velocities = new_vel
#     raw_positions = new_pos
#     # Print status periodically
#     if step % 50 == 0:
#         print(f"Step {step}: sample position of drone 0: {raw_positions[0]}")
#     time.sleep(0.01)

# print("Simulation complete.")
# import math
# from typing import Tuple

# def drag_force(velocity: float, drag_coefficient: float, area: float, density: float = 1.225) -> float:
#     return 0.5 * density * velocity**2 * drag_coefficient * area

# def decay_concentration(initial: float, rate: float, time_s: float) -> float:
#     return initial * math.exp(-rate * time_s)


# sim_core/simulation.py
# """
# Provides a simple simulation/test harness for the swarm core modules.
# Run this in test mode before live deployment.
# """
# import time
# import random
# from typing import List, Tuple
# from sensor_utils import get_corrected_position, fuse_gps_imu, get_range
# from flocking_controller import FlockingController
# from formation_planner import FormationPlanner

# # Simulation parameters
# NUM_DRONES = 10
# TIMESTEPS = 200
# DT = 0.1  # seconds per step

# # Initial random positions and zero velocities
# raw_positions: List[Tuple[float, float, float]] = [
#     (random.uniform(-5,5), random.uniform(-5,5), 10.0) for _ in range(NUM_DRONES)
# ]
# imu_deltas: List[Tuple[float, float, float]] = [(0.0,0.0,0.0)] * NUM_DRONES
# velocities: List[Tuple[float, float, float]] = [(0.0,0.0,0.0)] * NUM_DRONES

# # Generate a wall formation covering 3m target + 2m buffer
# targets = FormationPlanner.wall(NUM_DRONES, spacing=5.0, altitude=10.0)

# # Create controller
# flock = FlockingController(
#     perception_radius=6.0,
#     max_speed=3.0,
#     max_force=0.1,
#     nominal_spacing=5.0
# )

# # Run simulation
# for step in range(TIMESTEPS):
#     # Simulate IMU deltas (here zero or small noise)
#     imu_deltas = [(
#         random.gauss(0,0.01), random.gauss(0,0.01), random.gauss(0,0.005)
#     ) for _ in range(NUM_DRONES)]
#     # Compute accelerations
#     accs = flock.compute(
#         raw_positions,
#         imu_deltas,
#         velocities,
#         targets,
#         use_fusion=True
#     )
#     # Update velocities and positions
#     new_vel = []
#     new_pos = []
#     for i in range(NUM_DRONES):
#         vx, vy, vz = velocities[i]
#         ax, ay, az = accs[i]
#         # Euler integration
#         vx_new = vx + ax * DT
#         vy_new = vy + ay * DT
#         vz_new = vz + az * DT
#         # limit speed
#         speed = (vx_new**2 + vy_new**2 + vz_new**2)**0.5
#         if speed > flock.max_speed:
#             factor = flock.max_speed / speed
#             vx_new *= factor; vy_new *= factor; vz_new *= factor
#         x, y, z = raw_positions[i]
#         x_new = x + vx_new * DT
#         y_new = y + vy_new * DT
#         z_new = z + vz_new * DT
#         new_vel.append((vx_new, vy_new, vz_new))
#         new_pos.append((x_new, y_new, z_new))
#     velocities = new_vel
#     raw_positions = new_pos
#     # Print status periodically
#     if step % 50 == 0:
#         print(f"Step {step}:")
#         for i, pos in enumerate(raw_positions):
#             print(f"  Drone {i}: {pos}")
#     time.sleep(0.01)

# print("Simulation complete.")
# # Gemini
# sim_core/formation_planner.py
"""
Defines various formation target generators: wall, ring, ellipse, hex-grid, and now rectangle.
"""
import math
from typing import List, Tuple

class FormationPlanner:
    @staticmethod
    def wall(num: int, spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
        # Centered wall along x-axis
        start = -(num-1)/2 * spacing
        return [(start + i*spacing, 0.0, altitude) for i in range(num)]

    @staticmethod
    def ring(num: int, radius: float, altitude: float) -> List[Tuple[float, float, float]]:
        return [(
            math.cos(2*math.pi*i/num)*radius,
            math.sin(2*math.pi*i/num)*radius,
            altitude
        ) for i in range(num)]

    @staticmethod
    def ellipse(num: int, a: float, b: float, altitude: float) -> List[Tuple[float, float, float]]:
        return [(
            a*math.cos(2*math.pi*i/num),
            b*math.sin(2*math.pi*i/num),
            altitude
        ) for i in range(num)]

    @staticmethod
    def hex_grid(rows: int, cols: int, spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
        positions = []
        v_offset = math.sqrt(3)/2 * spacing
        # center grid
        width = (cols-1) * spacing
        height = (rows-1) * v_offset
        x0 = -width/2
        y0 = -height/2
        for i in range(rows):
            h_shift = (spacing/2) if (i%2) else 0.0
            for j in range(cols):
                x = x0 + j*spacing + h_shift
                y = y0 + i*v_offset
                positions.append((x, y, altitude))
        return positions

    @staticmethod
    def rectangle(num: int, width: float, height: float, altitude: float, aspect_ratio_preference: float = 1.0) -> List[Tuple[float, float, float]]:
        """
        Generates target positions for a rectangular formation.
        It tries to distribute drones somewhat evenly along the perimeter.
        Adjusts distribution based on number of drones and aspect ratio.
        """
        positions = []
        perimeter = 2 * (width + height)

        if num <= 0:
            return []
        
        # Distribute drones along the perimeter
        # Calculate segments for each side
        # Prioritize distributing along the longer sides more, but ensure at least 1 per side if possible
        num_on_width_side = max(1, round(num * (width / perimeter)))
        num_on_height_side = max(1, round(num * (height / perimeter)))

        # Ensure total drones match num, distribute remainder
        total_assigned = 2 * (num_on_width_side + num_on_height_side)
        remaining = num - total_assigned
        
        # Simple adjustment to ensure all drones are placed, might not be perfectly even for small N
        if remaining > 0:
            # Add to longer sides first
            if width >= height:
                num_on_width_side += math.ceil(remaining / 2)
                num_on_height_side += math.floor(remaining / 2)
            else:
                num_on_height_side += math.ceil(remaining / 2)
                num_on_width_side += math.floor(remaining / 2)

        # Ensure minimum 1 drone per corner, so at least 2 per side if N > 4
        # This simple distribution assumes enough drones for non-empty sides.
        # For small NUM_DRONES (e.g., 2 or 3), this might result in placing on only one side or corner.
        # A more robust solution for small N would directly place corners then fill.
        
        # Top side (x from -width/2 to width/2, y = height/2)
        if num_on_width_side > 0:
            for i in range(num_on_width_side):
                x = -width/2 + (width / (num_on_width_side + 1)) * (i + 1)
                positions.append((x, height/2, altitude))
        
        # Right side (y from height/2 to -height/2, x = width/2) - exclude top/bottom corners if already placed
        if num_on_height_side > 0:
            for i in range(num_on_height_side):
                y = height/2 - (height / (num_on_height_side + 1)) * (i + 1)
                positions.append((width/2, y, altitude))
        
        # Bottom side (x from width/2 to -width/2, y = -height/2) - exclude corners
        if num_on_width_side > 0:
            for i in range(num_on_width_side):
                x = width/2 - (width / (num_on_width_side + 1)) * (i + 1)
                positions.append((x, -height/2, altitude))

        # Left side (y from -height/2 to height/2, x = -width/2) - exclude corners
        if num_on_height_side > 0:
            for i in range(num_on_height_side):
                y = -height/2 + (height / (num_on_height_side + 1)) * (i + 1)
                positions.append((-width/2, y, altitude))
        
        # If num is smaller than total assigned, truncate. If larger, this simple
        # distribution might under-fill if `num_on_side` is not precisely calculated to sum to `num`.
        # For simplicity, we'll take the first `num` positions generated.
        return positions[:num]

