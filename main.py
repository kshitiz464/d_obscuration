# # main.py
# """
# Main script to orchestrate the drone swarm simulation and visualization.
# Allows for defining scenario parameters in one central place.
# """
# import math
# import random
# from typing import List, Tuple, Optional # Added Optional import

# # Import core simulation modules
# from sim_core.flocking_controller import FlockingController
# from sim_core.formation_planner import FormationPlanner
# from sim_core.sensor_utils import get_range # get_corrected_position is used internally by flocking_controller
# # from physics_utils import drag_force, decay_concentration # Uncomment if you want to use these
# # import spray_calculations # Uncomment if you have this module

# # Import the visualization function
# import sim_core.visualize

# def run_simulation_scenario(
#     num_drones: int,
#     timesteps: int,
#     dt: float,
#     horizontal_spacing: float,
#     vertical_spacing: float,
#     altitude: float,
#     min_base_wind_x: float,
#     max_base_wind_x: float,
#     min_base_wind_y: float,
#     max_base_wind_y: float,
#     min_base_wind_z: float,
#     max_base_wind_z: float,
#     wind_oscillation_amplitude_x: float,
#     wind_oscillation_amplitude_y: float,
#     wind_oscillation_frequency: float,
#     wind_oscillation_phase_y: float,
#     wind_gust_std_dev: float,
#     collision_threshold: float,
#     # Flocking Controller Parameters
#     perception_radius: float,
#     max_speed: float,
#     max_force: float,
#     weight_sep: float,
#     weight_align: float,
#     weight_cohesion: float,
#     weight_target: float,
#     kp_target: float,
#     nominal_spacing: float,
#     hover_slowing_radius: float,
#     seed: Optional[int] = None # For reproducible random values
# ) -> Tuple[List[List[List[float]]], int, List[Tuple[float, float, float]]]:
#     """
#     Runs a single simulation scenario with specified parameters.

#     Returns:
#         Tuple: (history of drone positions, total collisions, target positions)
#     """
#     if seed is not None:
#         random.seed(seed)

#     # Initial state for drones: scattered over a larger area, starting from low Z
#     positions: List[Tuple[float, float, float]] = [
#         (random.uniform(-70.0, 70.0),
#          random.uniform(-70.0, 70.0),
#          1.0) # All drones start from 1.0m altitude (ground)
#         for _ in range(num_drones)
#     ]
#     velocities: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * num_drones
#     imu_deltas: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * num_drones

#     # Generate staggered pattern targets
#     targets = FormationPlanner.staggered_pattern(num_drones, horizontal_spacing, vertical_spacing, altitude)

#     # Calculate the bounds of the target pattern for plot limits (used for perception_radius)
#     min_x_target = min(t[0] for t in targets)
#     max_x_target = max(t[0] for t in targets)
#     min_y_target = min(t[1] for t in targets)
#     max_y_target = max(t[1] for t in targets)

#     # Flocking controller configured for stable formation and hovering with wind compensation
#     flock = FlockingController(
#         perception_radius=perception_radius,
#         max_speed=max_speed,
#         max_force=max_force,
#         weight_sep=weight_sep,
#         weight_align=weight_align,
#         weight_cohesion=weight_cohesion,
#         weight_target=weight_target,
#         kp_target=kp_target,
#         nominal_spacing=nominal_spacing,
#         hover_slowing_radius=hover_slowing_radius
#     )

#     # Generate random base wind speeds for this run
#     current_base_wind_x = random.uniform(min_base_wind_x, max_base_wind_x)
#     current_base_wind_y = random.uniform(min_base_wind_y, max_base_wind_y)
#     current_base_wind_z = random.uniform(min_base_wind_z, max_base_wind_z)

#     # Initialize collision counter
#     total_collisions = 0

#     print(f"Running simulation for {num_drones} drones with random wind.")
#     print(f"Base Wind (X, Y, Z): ({current_base_wind_x:.2f}, {current_base_wind_y:.2f}, {current_base_wind_z:.2f}) m/s^2")
#     print(f"Targeting a staggered pattern with horizontal spacing {horizontal_spacing}m and vertical spacing {vertical_spacing}m at {altitude}m altitude.")
#     print("Generated target positions:")
#     for i, t in enumerate(targets):
#         print(f"  Drone {i}: Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

#     history = []

#     for step in range(timesteps):
#         # Simulate IMU deltas (noise for dead reckoning)
#         imu_deltas = [(
#             random.gauss(0, 0.005),
#             random.gauss(0, 0.005),
#             random.gauss(0, 0.002)
#         ) for _ in range(num_drones)]

#         # Calculate current wind acceleration
#         wind_x_osc = wind_oscillation_amplitude_x * math.sin(step * dt * wind_oscillation_frequency)
#         wind_y_osc = wind_oscillation_amplitude_y * math.sin(step * dt * wind_oscillation_frequency + wind_oscillation_phase_y)
        
#         wind_ax = current_base_wind_x + wind_x_osc + random.gauss(0, wind_gust_std_dev)
#         wind_ay = current_base_wind_y + wind_y_osc + random.gauss(0, wind_gust_std_dev)
#         wind_az = current_base_wind_z + random.gauss(0, wind_gust_std_dev * 0.5)

#         # Compute accelerations from flocking controller
#         accs = flock.compute(
#             positions,
#             imu_deltas,
#             velocities,
#             targets,
#             use_fusion=True
#         )

#         new_vel = []
#         new_pos = []
#         for i in range(num_drones):
#             vx, vy, vz = velocities[i]
#             ax, ay, az = accs[i]

#             # Add wind acceleration to the controller's acceleration
#             ax += wind_ax
#             ay += wind_ay
#             az += wind_az

#             # Euler integration for velocity update
#             vx_new = vx + ax * dt
#             vy_new = vy + ay * dt
#             vz_new = vz + az * dt

#             # Limit speed to the controller's max_speed
#             speed = math.sqrt(vx_new**2 + vy_new**2 + vz_new**2)
#             if speed > flock.max_speed:
#                 factor = flock.max_speed / speed
#                 vx_new *= factor
#                 vy_new *= factor
#                 vz_new *= factor
            
#             # Euler integration for position update
#             x, y, z = positions[i]
#             x_new = x + vx_new * dt
#             y_new = y + vy_new * dt
#             z_new = z + vz_new * dt
            
#             new_vel.append((vx_new, vy_new, vz_new))
#             new_pos.append((x_new, y_new, z_new))
        
#         velocities = new_vel
#         positions = new_pos

#         # --- Collision Detection ---
#         for i in range(num_drones):
#             for j in range(i + 1, num_drones): # Only check unique pairs
#                 dist = math.dist(positions[i], positions[j])
#                 if dist < collision_threshold:
#                     total_collisions += 1
#         # --- End Collision Detection ---

#         history.append([list(pos) for pos in positions])

#         if step % 50 == 0 or step == timesteps - 1:
#             print(f"\n--- Step {step} ---")
#             for i, pos in enumerate(positions):
#                 target_pos = targets[i]
#                 dist_to_target = math.sqrt(
#                     (pos[0] - target_pos[0])**2 +
#                     (pos[1] - target_pos[1])**2 +
#                     (pos[2] - target_pos[2])**2
#                 )
#                 print(f"  Drone {i}: Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), Dist to Target={dist_to_target:.2f}m")

#     print("\nSimulation complete.")
#     print(f"Total Collisions Detected: {total_collisions}")

#     return history, total_collisions, targets


# if __name__ == "__main__":
#     # --- Define Scenario Parameters Here ---
#     scenario_params = {
#         "num_drones": 7,
#         "timesteps": 3500,
#         "dt": 0.1,
#         "horizontal_spacing": 24.0,
#         "vertical_spacing": 6.0,
#         "altitude": 12.0,
#         # Wind Parameters
#         "min_base_wind_x": -0.05,
#         "max_base_wind_x": 0.15,
#         "min_base_wind_y": -0.05,
#         "max_base_wind_y": 0.15,
#         "min_base_wind_z": -0.02,
#         "max_base_wind_z": 0.02,
#         "wind_oscillation_amplitude_x": 0.05,
#         "wind_oscillation_amplitude_y": 0.03,
#         "wind_oscillation_frequency": 0.01,
#         "wind_oscillation_phase_y": math.pi / 2,
#         "wind_gust_std_dev": 0.01,
#         "collision_threshold": 1.0,
#         # Flocking Controller Parameters (tuned for robust navigation with wind)
#         "perception_radius": 50.0, # Adjust based on max expected formation spread + initial range
#         "max_speed": 10.0,
#         "max_force": 0.6,
#         "weight_sep": 4.0,
#         "weight_align": 1.0,
#         "weight_cohesion": 1.0,
#         "weight_target": 8.0,
#         "kp_target": 0.8,
#         "nominal_spacing": 5.0,
#         "hover_slowing_radius": 25.0,
#         "seed": None # Set to an integer for reproducible runs, or None for truly random
#     }

#     # Run the simulation
#     simulation_history, collisions, final_targets = run_simulation_scenario(**scenario_params)

#     # Visualize the results
#     sim_core.visualize.run_visualization(simulation_history, final_targets, scenario_params["altitude"])






# additional enhancements 



# main.py
"""
Main script to orchestrate the drone swarm simulation and visualization.
Now includes drone mass, gravity, and drag for more realistic physics.
Allows for defining scenario parameters in one central place.
"""
import math
import random
from typing import List, Tuple, Optional

# Import core simulation modules
from sim_core.flocking_controller import FlockingController
from sim_core.formation_planner import FormationPlanner
from sim_core.sensor_utils import get_range
from sim_core.physics_utils import drag_force # Import drag_force from physics_utils

# Import the visualization function
import sim_core.visualize

def run_simulation_scenario(
    num_drones: int,
    timesteps: int,
    dt: float,
    horizontal_spacing: float,
    vertical_spacing: float,
    altitude: float,
    min_base_wind_x: float,
    max_base_wind_x: float,
    min_base_wind_y: float,
    max_base_wind_y: float,
    min_base_wind_z: float,
    max_base_wind_z: float,
    wind_oscillation_amplitude_x: float,
    wind_oscillation_amplitude_y: float,
    wind_oscillation_frequency: float,
    wind_oscillation_phase_y: float,
    wind_gust_std_dev: float,
    collision_threshold: float,
    # New Physics Parameters
    drone_mass: float, # Mass of each drone in kg
    gravity_acceleration: float, # Acceleration due to gravity (e.g., -9.81 m/s^2)
    drag_coefficient: float, # Drag coefficient (Cd)
    drone_frontal_area: float, # Frontal area of the drone (A) in m^2
    # Flocking Controller Parameters
    perception_radius: float,
    max_speed: float,
    max_force: float, # Now represents max *force* in Newtons
    weight_sep: float,
    weight_align: float,
    weight_cohesion: float,
    weight_target: float,
    kp_target: float,
    nominal_spacing: float,
    hover_slowing_radius: float,
    vertical_hover_kp: float, # Proportional gain for vertical hover stability (N/m)
    vertical_hover_kd: float, # NEW: Derivative gain for vertical hover stability (N/(m/s))
    # Parameters with default values should come last
    air_density: float = 1.225, # Air density (rho) in kg/m^3 (standard)
    seed: Optional[int] = None # For reproducible random values
) -> Tuple[List[List[List[float]]], int, List[Tuple[float, float, float]]]:
    """
    Runs a single simulation scenario with specified parameters, including physics.

    Returns:
        Tuple: (history of drone positions, total collisions, target positions)
    """
    if seed is not None:
        random.seed(seed)

    # Initial state for drones: scattered over a larger area, starting from low Z
    positions: List[Tuple[float, float, float]] = [
        (random.uniform(-70.0, 70.0),
         random.uniform(-70.0, 70.0),
         1.0) # All drones start from 1.0m altitude (ground)
        for _ in range(num_drones)
    ]
    velocities: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * num_drones
    imu_deltas: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * num_drones

    # Generate staggered pattern targets
    targets = FormationPlanner.staggered_pattern(num_drones, horizontal_spacing, vertical_spacing, altitude)

    # Calculate the bounds of the target pattern for plot limits (used for perception_radius)
    min_x_target = min(t[0] for t in targets)
    max_x_target = max(t[0] for t in targets)
    min_y_target = min(t[1] for t in targets)
    max_y_target = max(t[1] for t in targets)

    # Flocking controller configured for stable formation and hovering with wind compensation
    # max_force for the controller is now interpreted as the maximum *force* (Newtons)
    flock = FlockingController(
        perception_radius=perception_radius,
        max_speed=max_speed,
        max_force=max_force, # Passed as force (Newtons)
        weight_sep=weight_sep,
        weight_align=weight_align,
        weight_cohesion=weight_cohesion, # Corrected from 'cohesion' to 'weight_cohesion'
        weight_target=weight_target,
        kp_target=kp_target,
        nominal_spacing=nominal_spacing,
        hover_slowing_radius=hover_slowing_radius,
        vertical_hover_kp=vertical_hover_kp, # Pass new parameter
        vertical_hover_kd=vertical_hover_kd # Pass NEW parameter
    )

    # Generate random base wind speeds for this run
    current_base_wind_x = random.uniform(min_base_wind_x, max_base_wind_x)
    current_base_wind_y = random.uniform(min_base_wind_y, max_base_wind_y)
    current_base_wind_z = random.uniform(min_base_wind_z, max_base_wind_z)

    # Initialize collision counter
    total_collisions = 0

    print(f"Running simulation for {num_drones} drones with random wind and physics.")
    print(f"Drone Mass: {drone_mass} kg, Gravity: {gravity_acceleration} m/s^2")
    print(f"Base Wind (X, Y, Z): ({current_base_wind_x:.2f}, {current_base_wind_y:.2f}, {current_base_wind_z:.2f}) m/s^2")
    print(f"Targeting a staggered pattern with horizontal spacing {horizontal_spacing}m and vertical spacing {vertical_spacing}m at {altitude}m altitude.")
    print("Generated target positions:")
    for i, t in enumerate(targets):
        print(f"  Drone {i}: Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

    history = []

    for step in range(timesteps):
        # Simulate IMU deltas (noise for dead reckoning)
        imu_deltas = [(
            random.gauss(0, 0.005),
            random.gauss(0, 0.005),
            random.gauss(0, 0.002)
        ) for _ in range(num_drones)]

        # Calculate current wind acceleration
        wind_x_osc = wind_oscillation_amplitude_x * math.sin(step * dt * wind_oscillation_frequency)
        wind_y_osc = wind_oscillation_amplitude_y * math.sin(step * dt * wind_oscillation_frequency + wind_oscillation_phase_y)
        
        wind_ax = current_base_wind_x + wind_x_osc + random.gauss(0, wind_gust_std_dev)
        wind_ay = current_base_wind_y + wind_y_osc + random.gauss(0, wind_gust_std_dev)
        wind_az = current_base_wind_z + random.gauss(0, wind_gust_std_dev * 0.5)

        # Compute forces from flocking controller (output `accs` are now interpreted as forces)
        flocking_forces = flock.compute(
            positions,
            imu_deltas,
            velocities,
            targets,
            use_fusion=True
        )

        new_vel = []
        new_pos = []
        for i in range(num_drones):
            vx, vy, vz = velocities[i]
            fx, fy, fz = flocking_forces[i] # These are now forces (Newtons) from the controller

            # --- Apply Physics ---
            # Convert flocking forces to acceleration
            ax_flock = fx / drone_mass
            ay_flock = fy / drone_mass
            az_flock = fz / drone_mass

            # Apply gravity
            az_gravity = gravity_acceleration # This value should be negative, e.g., -9.81
            
            # Calculate current speed for drag calculation
            current_speed = math.sqrt(vx**2 + vy**2 + vz**2)

            # Calculate drag force (which opposes motion)
            drag_fx, drag_fy, drag_fz = 0.0, 0.0, 0.0
            if current_speed > 1e-6: # Avoid division by zero if speed is negligible
                drag_magnitude = drag_force(current_speed, drag_coefficient, drone_frontal_area, air_density)
                # Drag acceleration is drag force / mass, acting opposite to velocity
                drag_ax = -(drag_magnitude / drone_mass) * (vx / current_speed)
                drag_ay = -(drag_magnitude / drone_mass) * (vy / current_speed)
                drag_az = -(drag_magnitude / drone_mass) * (vz / current_speed)
            else: # If drone is stationary, no drag
                drag_ax, drag_ay, drag_az = 0.0, 0.0, 0.0

            # Total acceleration on the drone
            total_ax = ax_flock + wind_ax + drag_ax
            total_ay = ay_flock + wind_ay + drag_ay
            total_az = az_flock + az_gravity + wind_az + drag_az

            # Euler integration for velocity update
            vx_new = vx + total_ax * dt
            vy_new = vy + total_ay * dt
            vz_new = vz + total_az * dt

            # Limit speed to the controller's max_speed
            speed_new = math.sqrt(vx_new**2 + vy_new**2 + vz_new**2)
            if speed_new > flock.max_speed:
                factor = flock.max_speed / speed_new
                vx_new *= factor
                vy_new *= factor
                vz_new *= factor
            
            # Euler integration for position update
            x, y, z = positions[i]
            x_new = x + vx_new * dt
            y_new = y + vy_new * dt
            z_new = z + vz_new * dt
            
            new_vel.append((vx_new, vy_new, vz_new))
            new_pos.append((x_new, y_new, z_new))
        
        velocities = new_vel
        positions = new_pos

        # --- Collision Detection ---
        for i in range(num_drones):
            for j in range(i + 1, num_drones): # Only check unique pairs
                dist = math.dist(positions[i], positions[j])
                if dist < collision_threshold:
                    total_collisions += 1
        # --- End Collision Detection ---

        history.append([list(pos) for pos in positions])

        if step % 50 == 0 or step == timesteps - 1:
            print(f"\n--- Step {step} ---")
            for i, pos in enumerate(positions):
                target_pos = targets[i]
                dist_to_target = math.sqrt(
                    (pos[0] - target_pos[0])**2 +
                    (pos[1] - target_pos[1])**2 +
                    (pos[2] - target_pos[2])**2
                )
                print(f"  Drone {i}: Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), Dist to Target={dist_to_target:.2f}m")

    print("\nSimulation complete.")
    print(f"Total Collisions Detected: {total_collisions}")

    return history, total_collisions, targets


if __name__ == "__main__":
    # --- Define Scenario Parameters Here ---
    scenario_params = {
        "num_drones": 7,
        "timesteps": 4000, # Increased timesteps to allow for complex physics to settle
        "dt": 0.1,
        "horizontal_spacing": 16.0,
        "vertical_spacing": 4.0,
        "altitude": 12.0,
        # Wind Parameters
        "min_base_wind_x": -0.05,
        "max_base_wind_x": 0.15,
        "min_base_wind_y": -0.05,
        "max_base_wind_y": 0.15,
        "min_base_wind_z": -0.02, # Random vertical wind component
        "max_base_wind_z": 0.02,
        "wind_oscillation_amplitude_x": 0.05,
        "wind_oscillation_amplitude_y": 0.03, # Y-oscillation
        "wind_oscillation_frequency": 0.01,
        "wind_oscillation_phase_y": math.pi / 2,
        "wind_gust_std_dev": 0.01,
        "collision_threshold": 1.0,
        # NEW Physics Parameters
        "drone_mass": 1.0, # kg - A typical small drone mass
        "gravity_acceleration": -9.81, # m/s^2 - Negative for downward acceleration
        "drag_coefficient": 0.5, # Unitless - Typical for bluff bodies
        "drone_frontal_area": 0.1, # m^2 - Estimated frontal area of a drone
        "air_density": 1.225, # kg/m^3 - Standard air density
        # Flocking Controller Parameters (tuned for robust navigation with wind & physics)
        "perception_radius": 50.0, # Adjust based on max expected formation spread + initial range
        "max_speed": 10.0,
        "max_force": 25.0, # Newtons - Increased max force to provide more thrust margin
        "weight_sep": 4.0,
        "weight_align": 1.0,
        "weight_cohesion": 1.0,
        "weight_target": 8.0,
        "kp_target": 0.9, # Slightly increased kp_target for more assertive target seeking
        "nominal_spacing": 5.0,
        "hover_slowing_radius": 25.0,
        "vertical_hover_kp": 25.0, # Proportional gain for vertical hover stability (N/m) - Further Adjusted
        "vertical_hover_kd": 40.0, # Derivative gain for vertical hover stability (N/(m/s)) - Further Increased
        "seed": None # Set to an integer for reproducible runs, or None for truly random
    }

    # Run the simulation
    simulation_history, collisions, final_targets = run_simulation_scenario(**scenario_params)

    # Visualize the results
    sim_core.visualize.run_visualization(simulation_history, final_targets, scenario_params["altitude"])
