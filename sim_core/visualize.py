
# # approach 4




# # sim_core/visualize.py
# """
# Visualize drone movement forming a staggered pattern in 3D using Matplotlib animation.
# Now uses the precise staggered pattern logic from FormationPlanner.
# Adjusted parameters for stable flocking and hovering around targets.
# Drones now start from a wider range of initial positions to test robust navigation.
# """
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from mpl_toolkits.mplot3d import Axes3D
# from flocking_controller import FlockingController # Import the updated FlockingController
# from formation_planner import FormationPlanner # Import the updated FormationPlanner
# import random
# import json
# import math
# from sensor_utils import get_range # Only need get_range explicitly here if not for debug prints

# # Simulation constants
# NUM_DRONES = 7 # Example for the 4+3 staggered pattern from the image
# TIMESTEPS = 3500 # Increased steps significantly for long-term stability and initial movement
# DT = 0.1

# # Staggered pattern formation parameters
# HORIZONTAL_SPACING = 9.0 # Horizontal distance between drones in a row
# VERTICAL_SPACING = 4.0   # Vertical distance between the two rows
# ALTITUDE = 12.0          # Flight height above ground

# # Initial state for drones: scattered over a larger area, starting from low Z
# positions = [
#     (random.uniform(-20.0, 20.0),  # Wider random spread for X
#      random.uniform(-20.0, 20.0),  # Wider random spread for Y
#      1.0)                        # All drones start from 1.0m altitude (ground)
#     for _ in range(NUM_DRONES)
# ]
# velocities = [(0.0, 0.0, 0.0)] * NUM_DRONES
# imu_deltas = [(0.0, 0.0, 0.0)] * NUM_DRONES # Will be updated in loop

# # Generate staggered pattern targets using the FormationPlanner
# targets = FormationPlanner.staggered_pattern(NUM_DRONES, HORIZONTAL_SPACING, VERTICAL_SPACING, ALTITUDE)

# # Calculate the bounds of the target pattern for plot limits
# min_x_target = min(t[0] for t in targets)
# max_x_target = max(t[0] for t in targets)
# min_y_target = min(t[1] for t in targets)
# max_y_target = max(t[1] for t in targets)

# # Flocking controller configured for stable formation and hovering
# # Parameters adjusted for more robust navigation from arbitrary starting points
# flock = FlockingController(
#     perception_radius=max(max_x_target - min_x_target, max_y_target - min_y_target) * 0.75 + 20.0, # Increased perception further
#     max_speed=9.0,     # Significantly increased max_speed for faster long-distance travel
#     max_force=0.6,      # Significantly increased max_force for stronger initial acceleration
#     weight_sep=5.0,     # Increased separation weight even more for stronger collision avoidance
#     weight_align=1.0,   # Enable alignment
#     weight_cohesion=1.0,# Enable cohesion
#     weight_target=8.0,  # Even stronger target attraction to pull them from far away
#     kp_target=0.8,      # Increased base proportional gain for target steering
#     nominal_spacing=5.0, # Desired spacing when flocking
#     hover_slowing_radius=25.0 # Drones start slowing target attraction when within 25m of target
# )

# print(f"Visualization simulation started for {NUM_DRONES} drones.")
# print(f"Targeting a staggered pattern with horizontal spacing {HORIZONTAL_SPACING}m and vertical spacing {VERTICAL_SPACING}m at {ALTITUDE}m altitude.")
# print("Generated target positions:")
# for i, t in enumerate(targets):
#     print(f"  Drone {i}: Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

# # History for animation and logging
# history = []

# for step in range(TIMESTEPS):
#     # Simulate small IMU deltas (noise for dead reckoning)
#     imu_deltas = [(
#         random.gauss(0, 0.005), 
#         random.gauss(0, 0.005), 
#         random.gauss(0, 0.002)
#     ) for _ in range(NUM_DRONES)]

#     # Compute accelerations using the flocking controller
#     accs = flock.compute(
#         positions,
#         imu_deltas,
#         velocities,
#         targets,
#         use_fusion=True
#     )

#     new_vel = []
#     new_pos = []
#     for i in range(NUM_DRONES):
#         vx, vy, vz = velocities[i]
#         ax, ay, az = accs[i]

#         # Euler integration for velocity update
#         vx_new = vx + ax * DT
#         vy_new = vy + ay * DT
#         vz_new = vz + az * DT

#         # Limit speed to the controller's max_speed
#         speed = math.sqrt(vx_new**2 + vy_new**2 + vz_new**2)
#         if speed > flock.max_speed:
#             factor = flock.max_speed / speed
#             vx_new *= factor
#             vy_new *= factor
#             vz_new *= factor
        
#         # Euler integration for position update
#         x, y, z = positions[i]
#         x_new = x + vx_new * DT
#         y_new = y + vy_new * DT
#         z_new = z + vz_new * DT
        
#         new_vel.append((vx_new, vy_new, vz_new))
#         new_pos.append((x_new, y_new, z_new))
    
#     velocities = new_vel
#     positions = new_pos
#     history.append([list(pos) for pos in positions]) # Store current positions for this timestep

#     if step % 50 == 0 or step == TIMESTEPS - 1:
#         print(f"\n--- Step {step} ---")
#         for i, pos in enumerate(positions):
#             target_pos = targets[i]
#             dist_to_target = math.sqrt(
#                 (pos[0] - target_pos[0])**2 + 
#                 (pos[1] - target_pos[1])**2 + 
#                 (pos[2] - target_pos[2])**2
#             )
#             print(f"  Drone {i}: Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), Dist to Target={dist_to_target:.2f}m")

# print("\nSimulation complete. Generating visualization...")

# # Save log to file (optional, for external analysis)
# with open("visualize_drone_log.json", "w") as f:
#     json.dump(history, f)

# # 3D Visualization setup
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# # Drone scatter plot (will be updated in animation)
# scat = ax.scatter([], [], [], s=80, c='blue', label='Drones')

# # Target scatter plot (static, 'X' markers for clarity)
# txs = [t[0] for t in targets]
# tys = [t[1] for t in targets]
# tzs = [t[2] for t in targets]
# ax.scatter(txs, tys, tzs, s=100, c='red', marker='X', label='Targets')

# # Frontal view: camera at y-positive looking toward negative y. Adjust 'azim' for different views.
# ax.view_init(elev=20, azim=180) 

# # Set dynamic axes limits based on calculated target pattern bounds and initial spread
# initial_min_x = min(p[0] for p in history[0])
# initial_max_x = max(p[0] for p in history[0])
# initial_min_y = min(p[1] for p in history[0])
# initial_max_y = max(p[1] for p in history[0])
# initial_min_z = min(p[2] for p in history[0])
# initial_max_z = max(p[2] for p in history[0])

# # Combine initial and target bounds for comprehensive limits
# overall_min_x = min(min_x_target, initial_min_x)
# overall_max_x = max(max_x_target, initial_max_x)
# overall_min_y = min(min_y_target, initial_min_y)
# overall_max_y = max(max_y_target, initial_max_y)
# # For Z, ensure it covers ground (1.0) up to max initial Z and target altitude + padding
# overall_min_z = 1.0 # Drones start at 1m
# overall_max_z = max(ALTITUDE + 5, initial_max_z + 5) # Pad initial max_z and target altitude

# padding_factor_x = 0.3 # Increased padding for wider initial spread
# padding_factor_y = 0.3
# padding_factor_z = 0.3

# ax.set_xlim(overall_min_x - (overall_max_x - overall_min_x) * padding_factor_x, overall_max_x + (overall_max_x - overall_min_x) * padding_factor_x)
# ax.set_ylim(overall_min_y - (overall_max_y - overall_min_y) * padding_factor_y, overall_max_y + (overall_max_y - overall_min_y) * padding_factor_y)
# ax.set_zlim(overall_min_z - 1, overall_max_z + 1) # Small padding around ground and max altitude


# ax.set_title("Drone Swarm - Staggered Pattern Formation (Robust Navigation)")
# ax.set_xlabel("X (meters)")
# ax.set_ylabel("Y (meters)")
# ax.set_zlabel("Z (Altitude, meters)")
# ax.legend()
# ax.grid(True)

# # Animation function to update drone positions in each frame
# def update(frame):
#     coords = history[frame]
#     xs = [x for x, y, z in coords]
#     ys = [y for x, y, z in coords]
#     zs = [z for x, y, z in coords]
#     scat._offsets3d = (xs, ys, zs)
#     return scat,

# # Create the animation
# ani = animation.FuncAnimation(fig, update, frames=len(history), interval=50, blit=False)

# # To display the animation, you need to call plt.show()
# plt.show()




# # another approach 


# # sim_core/visualize.py
# """
# Visualize drone movement forming a staggered pattern in 3D using Matplotlib animation.
# Includes a wind factor (time-varying and gusts) with random base speeds for added realism.
# Adjusted parameters for robust navigation and stable hovering with wind.
# Now includes a collision counter.
# Plot limits are dynamically adjusted to show the entire simulation trajectory.
# """
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from mpl_toolkits.mplot3d import Axes3D
# from flocking_controller import FlockingController
# from formation_planner import FormationPlanner
# import random
# import json
# import math
# from sensor_utils import get_range # Only need get_range explicitly here if not for debug prints

# # Simulation constants
# NUM_DRONES = 7
# TIMESTEPS = 4000 # Increased steps for observing behavior with wind
# DT = 0.1

# # Staggered pattern formation parameters
# HORIZONTAL_SPACING = 22.0
# VERTICAL_SPACING = 20.0
# ALTITUDE = 12.0

# # Wind parameters - Now with ranges for random base speeds
# MIN_BASE_WIND_X = -0.05 # Minimum constant wind component in X direction (m/s^2)
# MAX_BASE_WIND_X = 0.15  # Maximum constant wind component in X direction (m/s^2)
# MIN_BASE_WIND_Y = -0.05 # Minimum constant wind component in Y direction (m/s^2)
# MAX_BASE_WIND_Y = 0.15  # Maximum constant wind component in Y direction (m/s^2)

# WIND_OSCILLATION_AMPLITUDE_X = 0.05 # Amplitude of sinusoidal wind oscillation X (m/s^2)
# WIND_OSCILLATION_FREQUENCY = 0.01 # Frequency of wind oscillation
# WIND_GUST_STD_DEV = 0.01 # Standard deviation for random gusts (m/s^2)

# # Collision detection parameter
# COLLISION_THRESHOLD = 1.0 # meters. If two drones are closer than this, it's considered a collision.

# # Initial state for drones: scattered over a larger area, starting from low Z
# positions = [
#     (random.uniform(-70.0, 70.0),
#      random.uniform(-70.0, 70.0),
#      1.0) # All drones start from 1.0m altitude (ground)
#     for _ in range(NUM_DRONES)
# ]
# velocities = [(0.0, 0.0, 0.0)] * NUM_DRONES
# imu_deltas = [(0.0, 0.0, 0.0)] * NUM_DRONES

# # Generate staggered pattern targets
# targets = FormationPlanner.staggered_pattern(NUM_DRONES, HORIZONTAL_SPACING, VERTICAL_SPACING, ALTITUDE)

# # Calculate the bounds of the target pattern for plot limits (used for perception_radius)
# min_x_target = min(t[0] for t in targets)
# max_x_target = max(t[0] for t in targets)
# min_y_target = min(t[1] for t in targets)
# max_y_target = max(t[1] for t in targets)

# # Flocking controller configured for stable formation and hovering with wind compensation
# flock = FlockingController(
#     perception_radius=max(max_x_target - min_x_target, max_y_target - min_y_target) * 0.75 + 30.0,
#     max_speed=10.0,
#     max_force=0.6,      # Increased max_force to better fight wind
#     weight_sep=4.0,     # Further increased separation weight for better collision avoidance with disturbances
#     weight_align=1.0,
#     weight_cohesion=1.0,
#     weight_target=8.0, # Stronger target attraction to counter wind drift
#     kp_target=0.8,      # Increased base proportional gain for target steering
#     nominal_spacing=5.0,
#     hover_slowing_radius=25.0
# )

# # Generate random base wind speeds for this run
# current_base_wind_x = random.uniform(MIN_BASE_WIND_X, MAX_BASE_WIND_X)
# current_base_wind_y = random.uniform(MIN_BASE_WIND_Y, MAX_BASE_WIND_Y)

# # Initialize collision counter
# total_collisions = 0

# print(f"Visualization simulation started for {NUM_DRONES} drones with random wind.")
# print(f"Base Wind (X, Y): ({current_base_wind_x:.2f}, {current_base_wind_y:.2f}) m/s^2")
# print(f"Targeting a staggered pattern with horizontal spacing {HORIZONTAL_SPACING}m and vertical spacing {VERTICAL_SPACING}m at {ALTITUDE}m altitude.")
# print("Generated target positions:")
# for i, t in enumerate(targets):
#     print(f"  Drone {i}: Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

# history = []

# for step in range(TIMESTEPS):
#     # Simulate IMU deltas (noise for dead reckoning)
#     imu_deltas = [(
#         random.gauss(0, 0.005),
#         random.gauss(0, 0.005),
#         random.gauss(0, 0.002)
#     ) for _ in range(NUM_DRONES)]

#     # Calculate current wind acceleration
#     # Time-varying wind component (sinusoidal)
#     wind_x_osc = WIND_OSCILLATION_AMPLITUDE_X * math.sin(step * DT * WIND_OSCILLATION_FREQUENCY)
    
#     # Use the randomly generated base wind components
#     wind_ax = current_base_wind_x + wind_x_osc + random.gauss(0, WIND_GUST_STD_DEV)
#     wind_ay = current_base_wind_y + random.gauss(0, WIND_GUST_STD_DEV)
#     wind_az = 0.0 + random.gauss(0, WIND_GUST_STD_DEV * 0.5) # Small vertical gusts

#     # Compute accelerations from flocking controller
#     accs = flock.compute(
#         positions,
#         imu_deltas,
#         velocities,
#         targets,
#         use_fusion=True
#     )

#     new_vel = []
#     new_pos = []
#     for i in range(NUM_DRONES):
#         vx, vy, vz = velocities[i]
#         ax, ay, az = accs[i]

#         # Add wind acceleration to the controller's acceleration
#         ax += wind_ax
#         ay += wind_ay
#         az += wind_az

#         # Euler integration for velocity update
#         vx_new = vx + ax * DT
#         vy_new = vy + ay * DT
#         vz_new = vz + az * DT

#         # Limit speed to the controller's max_speed
#         speed = math.sqrt(vx_new**2 + vy_new**2 + vz_new**2)
#         if speed > flock.max_speed:
#             factor = flock.max_speed / speed
#             vx_new *= factor
#             vy_new *= factor
#             vz_new *= factor
        
#         # Euler integration for position update
#         x, y, z = positions[i]
#         x_new = x + vx_new * DT
#         y_new = y + vy_new * DT
#         z_new = z + vz_new * DT
        
#         new_vel.append((vx_new, vy_new, vz_new))
#         new_pos.append((x_new, y_new, z_new))
    
#     velocities = new_vel
#     positions = new_pos

#     # --- Collision Detection ---
#     for i in range(NUM_DRONES):
#         for j in range(i + 1, NUM_DRONES): # Only check unique pairs
#             dist = math.dist(positions[i], positions[j])
#             if dist < COLLISION_THRESHOLD:
#                 total_collisions += 1
#     # --- End Collision Detection ---

#     history.append([list(pos) for pos in positions])

#     if step % 50 == 0 or step == TIMESTEPS - 1:
#         print(f"\n--- Step {step} ---")
#         for i, pos in enumerate(positions):
#             target_pos = targets[i]
#             dist_to_target = math.sqrt(
#                 (pos[0] - target_pos[0])**2 +
#                 (pos[1] - target_pos[1])**2 +
#                 (pos[2] - target_pos[2])**2
#             )
#             print(f"  Drone {i}: Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), Dist to Target={dist_to_target:.2f}m")

# print("\nSimulation complete. Generating visualization...")
# print(f"Total Collisions Detected: {total_collisions}")

# with open("visualize_drone_log.json", "w") as f:
#     json.dump(history, f)

# # 3D Visualization setup
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# scat = ax.scatter([], [], [], s=80, c='blue', label='Drones')

# txs = [t[0] for t in targets]
# tys = [t[1] for t in targets]
# tzs = [t[2] for t in targets]
# ax.scatter(txs, tys, tzs, s=100, c='red', marker='X', label='Targets')

# ax.view_init(elev=20, azim=180) 

# # --- Dynamic Plot Limits based on entire history ---
# all_xs = [pos[0] for frame in history for pos in frame]
# all_ys = [pos[1] for frame in history for pos in frame]
# all_zs = [pos[2] for frame in history for pos in frame]

# # Add target positions to ensure they are also included in the bounds
# all_xs.extend(txs)
# all_ys.extend(tys)
# all_zs.extend(tzs)

# if all_xs: # Ensure lists are not empty
#     min_overall_x = min(all_xs)
#     max_overall_x = max(all_xs)
#     min_overall_y = min(all_ys)
#     max_overall_y = max(all_ys)
#     min_overall_z = min(all_zs)
#     max_overall_z = max(all_zs)
# else: # Fallback if history is empty
#     min_overall_x, max_overall_x = -100, 100
#     min_overall_y, max_overall_y = -100, 100
#     min_overall_z, max_overall_z = 0, 50


# padding_factor = 0.1 # Add 10% padding to the overall min/max for better visibility
# range_x = max_overall_x - min_overall_x
# range_y = max_overall_y - min_overall_y
# range_z = max_overall_z - min_overall_z

# ax.set_xlim(min_overall_x - range_x * padding_factor, max_overall_x + range_x * padding_factor)
# ax.set_ylim(min_overall_y - range_y * padding_factor, max_overall_y + range_y * padding_factor)
# ax.set_zlim(min_overall_z - range_z * padding_factor, max_overall_z + range_z * padding_factor) # Ensure z-axis starts near ground if drones go low


# ax.set_title("Drone Swarm - Staggered Pattern Formation with Wind")
# ax.set_xlabel("X (meters)")
# ax.set_ylabel("Y (meters)")
# ax.set_zlabel("Z (Altitude, meters)")
# ax.legend()
# ax.grid(True)

# def update(frame):
#     coords = history[frame]
#     xs = [x for x, y, z in coords]
#     ys = [y for x, y, z in coords]
#     zs = [z for x, y, z in coords]
#     scat._offsets3d = (xs, ys, zs)
#     return scat,

# ani = animation.FuncAnimation(fig, update, frames=len(history), interval=50, blit=False)
# plt.show()





























## harder wind test 




# # sim_core/visualize.py
# """
# Visualize drone movement forming a staggered pattern in 3D using Matplotlib animation.
# Includes a more realistic wind factor (random base speeds, oscillations in X/Y, and gusts).
# Adjusted parameters for robust navigation and stable hovering with enhanced wind realism.
# Now includes a collision counter.
# Plot limits are dynamically adjusted to show the entire simulation trajectory.
# """
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# from mpl_toolkits.mplot3d import Axes3D
# from flocking_controller import FlockingController
# from formation_planner import FormationPlanner
# import random
# import json
# import math
# from sensor_utils import get_range # Only need get_range explicitly here if not for debug prints

# # Simulation constants
# NUM_DRONES = 7
# TIMESTEPS = 4000 # Increased steps for observing behavior with wind
# DT = 0.1

# # Staggered pattern formation parameters
# HORIZONTAL_SPACING = 10.0
# VERTICAL_SPACING = 8.0
# ALTITUDE = 12.0

# # Wind parameters - Now with ranges for random base speeds and Y-oscillation
# MIN_BASE_WIND_X = -0.05 # Minimum constant wind component in X direction (m/s^2)
# MAX_BASE_WIND_X = 0.15  # Maximum constant wind component in X direction (m/s^2)
# MIN_BASE_WIND_Y = -0.05 # Minimum constant wind component in Y direction (m/s^2)
# MAX_BASE_WIND_Y = 0.15  # Maximum constant wind component in Y direction (m/s^2)
# MIN_BASE_WIND_Z = -0.02 # Minimum constant wind component in Z direction (m/s^2)
# MAX_BASE_WIND_Z = 0.02  # Maximum constant wind component in Z direction (m/s^2)

# WIND_OSCILLATION_AMPLITUDE_X = 0.05 # Amplitude of sinusoidal wind oscillation X (m/s^2)
# WIND_OSCILLATION_AMPLITUDE_Y = 0.03 # Amplitude of sinusoidal wind oscillation Y (m/s^2)
# WIND_OSCILLATION_FREQUENCY = 0.01 # Frequency for both X and Y wind oscillations
# WIND_OSCILLATION_PHASE_Y = math.pi / 2 # Phase offset for Y oscillation (90 degrees out of phase with X)
# WIND_GUST_STD_DEV = 0.01 # Standard deviation for random gusts (m/s^2)

# # Collision detection parameter
# COLLISION_THRESHOLD = 1.0 # meters. If two drones are closer than this, it's considered a collision.

# # Initial state for drones: scattered over a larger area, starting from low Z
# positions = [
#     (random.uniform(-70.0, 70.0),
#      random.uniform(-70.0, 70.0),
#      1.0) # All drones start from 1.0m altitude (ground)
#     for _ in range(NUM_DRONES)
# ]
# velocities = [(0.0, 0.0, 0.0)] * NUM_DRONES
# imu_deltas = [(0.0, 0.0, 0.0)] * NUM_DRONES

# # Generate staggered pattern targets
# targets = FormationPlanner.staggered_pattern(NUM_DRONES, HORIZONTAL_SPACING, VERTICAL_SPACING, ALTITUDE)

# # Calculate the bounds of the target pattern for plot limits (used for perception_radius)
# min_x_target = min(t[0] for t in targets)
# max_x_target = max(t[0] for t in targets)
# min_y_target = min(t[1] for t in targets)
# max_y_target = max(t[1] for t in targets)

# # Flocking controller configured for stable formation and hovering with wind compensation
# flock = FlockingController(
#     perception_radius=max(max_x_target - min_x_target, max_y_target - min_y_target) * 0.75 + 30.0,
#     max_speed=10.0,
#     max_force=0.7,      # Increased max_force to better fight wind and be more responsive
#     weight_sep=4.0,     # Further increased separation weight for better collision avoidance with disturbances
#     weight_align=1.0,
#     weight_cohesion=1.0,
#     weight_target=9.0, # Stronger target attraction to counter wind drift
#     kp_target=0.9,      # Increased base proportional gain for target steering
#     nominal_spacing=5.0,
#     hover_slowing_radius=25.0
# )

# # Generate random base wind speeds for this run
# current_base_wind_x = random.uniform(MIN_BASE_WIND_X, MAX_BASE_WIND_X)
# current_base_wind_y = random.uniform(MIN_BASE_WIND_Y, MAX_BASE_WIND_Y)
# current_base_wind_z = random.uniform(MIN_BASE_WIND_Z, MAX_BASE_WIND_Z)

# # Initialize collision counter
# total_collisions = 0 # Moved initialization here

# print(f"Visualization simulation started for {NUM_DRONES} drones with random wind.")
# print(f"Base Wind (X, Y, Z): ({current_base_wind_x:.2f}, {current_base_wind_y:.2f}, {current_base_wind_z:.2f}) m/s^2")
# print(f"Targeting a staggered pattern with horizontal spacing {HORIZONTAL_SPACING}m and vertical spacing {VERTICAL_SPACING}m at {ALTITUDE}m altitude.")
# print("Generated target positions:")
# for i, t in enumerate(targets):
#     print(f"  Drone {i}: Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

# history = []

# for step in range(TIMESTEPS):
#     # Simulate IMU deltas (noise for dead reckoning)
#     imu_deltas = [(
#         random.gauss(0, 0.005),
#         random.gauss(0, 0.005),
#         random.gauss(0, 0.002)
#     ) for _ in range(NUM_DRONES)]

#     # Calculate current wind acceleration
#     # Time-varying wind components (sinusoidal)
#     wind_x_osc = WIND_OSCILLATION_AMPLITUDE_X * math.sin(step * DT * WIND_OSCILLATION_FREQUENCY)
#     wind_y_osc = WIND_OSCILLATION_AMPLITUDE_Y * math.sin(step * DT * WIND_OSCILLATION_FREQUENCY + WIND_OSCILLATION_PHASE_Y)
    
#     # Use the randomly generated base wind components + oscillation + gusts
#     wind_ax = current_base_wind_x + wind_x_osc + random.gauss(0, WIND_GUST_STD_DEV)
#     wind_ay = current_base_wind_y + wind_y_osc + random.gauss(0, WIND_GUST_STD_DEV)
#     wind_az = current_base_wind_z + random.gauss(0, WIND_GUST_STD_DEV * 0.5) # Small vertical gusts

#     # Compute accelerations from flocking controller
#     accs = flock.compute(
#         positions,
#         imu_deltas,
#         velocities,
#         targets,
#         use_fusion=True
#     )

#     new_vel = []
#     new_pos = []
#     for i in range(NUM_DRONES):
#         vx, vy, vz = velocities[i]
#         ax, ay, az = accs[i]

#         # Add wind acceleration to the controller's acceleration
#         ax += wind_ax
#         ay += wind_ay
#         az += wind_az

#         # Euler integration for velocity update
#         vx_new = vx + ax * DT
#         vy_new = vy + ay * DT
#         vz_new = vz + az * DT

#         # Limit speed to the controller's max_speed
#         speed = math.sqrt(vx_new**2 + vy_new**2 + vz_new**2)
#         if speed > flock.max_speed:
#             factor = flock.max_speed / speed
#             vx_new *= factor
#             vy_new *= factor
#             vz_new *= factor
        
#         # Euler integration for position update
#         x, y, z = positions[i]
#         x_new = x + vx_new * DT
#         y_new = y + vy_new * DT
#         z_new = z + vz_new * DT
        
#         new_vel.append((vx_new, vy_new, vz_new))
#         new_pos.append((x_new, y_new, z_new))
    
#     velocities = new_vel
#     positions = new_pos

#     # --- Collision Detection ---
#     for i in range(NUM_DRONES):
#         for j in range(i + 1, NUM_DRONES): # Only check unique pairs
#             dist = math.dist(positions[i], positions[j])
#             if dist < COLLISION_THRESHOLD:
#                 total_collisions += 1
#     # --- End Collision Detection ---

#     history.append([list(pos) for pos in positions])

#     if step % 50 == 0 or step == TIMESTEPS - 1:
#         print(f"\n--- Step {step} ---")
#         for i, pos in enumerate(positions):
#             target_pos = targets[i]
#             dist_to_target = math.sqrt(
#                 (pos[0] - target_pos[0])**2 +
#                 (pos[1] - target_pos[1])**2 +
#                 (pos[2] - target_pos[2])**2
#             )
#             print(f"  Drone {i}: Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), Dist to Target={dist_to_target:.2f}m")

# print("\nSimulation complete. Generating visualization...")
# print(f"Total Collisions Detected: {total_collisions}")

# with open("visualize_drone_log.json", "w") as f:
#     json.dump(history, f)

# # 3D Visualization setup
# fig = plt.figure(figsize=(10, 8))
# ax = fig.add_subplot(111, projection='3d')

# scat = ax.scatter([], [], [], s=80, c='blue', label='Drones')

# txs = [t[0] for t in targets]
# tys = [t[1] for t in targets]
# tzs = [t[2] for t in targets]
# ax.scatter(txs, tys, tzs, s=100, c='red', marker='X', label='Targets')

# ax.view_init(elev=20, azim=180) 

# # --- Dynamic Plot Limits based on entire history ---
# all_xs = [pos[0] for frame in history for pos in frame]
# all_ys = [pos[1] for frame in history for pos in frame]
# all_zs = [pos[2] for frame in history for pos in frame]

# # Add target positions to ensure they are also included in the bounds
# all_xs.extend(txs)
# all_ys.extend(tys)
# all_zs.extend(tzs)

# if all_xs: # Ensure lists are not empty
#     min_overall_x = min(all_xs)
#     max_overall_x = max(all_xs)
#     min_overall_y = min(all_ys)
#     max_overall_y = max(all_ys)
#     min_overall_z = min(all_zs)
#     max_overall_z = max(all_zs)
# else: # Fallback if history is empty
#     min_overall_x, max_overall_x = -100, 100
#     min_overall_y, max_overall_y = -100, 100
#     min_overall_z, max_overall_z = 0, 50


# padding_factor = 0.1 # Add 10% padding to the overall min/max for better visibility
# range_x = max_overall_x - min_overall_x
# range_y = max_overall_y - min_overall_y
# range_z = max_overall_z - min_overall_z

# ax.set_xlim(min_overall_x - range_x * padding_factor, max_overall_x + range_x * padding_factor)
# ax.set_ylim(min_overall_y - range_y * padding_factor, max_overall_y + range_y * padding_factor)
# ax.set_zlim(min_overall_z - range_z * padding_factor, max_overall_z + range_z * padding_factor) # Ensure z-axis starts near ground if drones go low


# ax.set_title("Drone Swarm - Staggered Pattern Formation with Wind")
# ax.set_xlabel("X (meters)")
# ax.set_ylabel("Y (meters)")
# ax.set_zlabel("Z (Altitude, meters)")
# ax.legend()
# ax.grid(True)

# def update(frame):
#     coords = history[frame]
#     xs = [x for x, y, z in coords]
#     ys = [y for x, y, z in coords]
#     zs = [z for x, y, z in coords]
#     scat._offsets3d = (xs, ys, zs)
#     return scat,

# ani = animation.FuncAnimation(fig, update, frames=len(history), interval=50, blit=False)
# plt.show()













## Reusable code which can be accessed by main.py












# sim_core/visualize.py
"""
Visualization module for drone swarm simulation.
Contains a function to display the 3D drone movement based on provided simulation history.
"""
from typing import List, Tuple
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
import math
import json # Still needed if you write/read log files, but not for core visualization function

def run_visualization(history: List[List[List[float]]], targets: List[Tuple[float, float, float]], target_altitude: float):
    """
    Generates a 3D animation of drone movement and target positions.

    Args:
        history (List[List[List[float]]]): A list where each element is a timestep,
                                           and each timestep contains a list of [x, y, z] positions for all drones.
        targets (List[Tuple[float, float, float]]): A list of (x, y, z) target positions for each drone.
        target_altitude (float): The target altitude for the formation (used for z-axis limits).
    """

    # 3D Visualization setup
    fig = plt.figure(figsize=(10, 8))
    ax = fig.add_subplot(111, projection='3d')

    # Drone scatter plot (will be updated in animation)
    scat = ax.scatter([], [], [], s=80, c='blue', label='Drones') # type: ignore

    # Target scatter plot (static, 'X' markers for clarity)
    txs = [t[0] for t in targets]
    tys = [t[1] for t in targets]
    tzs = [t[2] for t in targets]
    ax.scatter(txs, tys, tzs, s=100, c='red', marker='X', label='Targets') # type: ignore

    # Frontal view: camera at y-positive looking toward negative y. Adjust 'azim' for different views.
    ax.view_init(elev=20, azim=180)  # type: ignore

    # --- Dynamic Plot Limits based on entire history ---
    all_xs = [pos[0] for frame in history for pos in frame]
    all_ys = [pos[1] for frame in history for pos in frame]
    all_zs = [pos[2] for frame in history for pos in frame]

    # Add target positions to ensure they are also included in the bounds
    all_xs.extend(txs)
    all_ys.extend(tys)
    all_zs.extend(tzs)

    # Ensure lists are not empty before calculating min/max
    if all_xs: 
        min_overall_x = min(all_xs)
        max_overall_x = max(all_xs)
        min_overall_y = min(all_ys)
        max_overall_y = max(all_ys)
        min_overall_z = min(all_zs)
        max_overall_z = max(all_zs)
    else: # Fallback if history is empty
        min_overall_x, max_overall_x = -100, 100
        min_overall_y, max_overall_y = -100, 100
        min_overall_z, max_overall_z = 0, 50


    padding_factor_x = 0.1 # Add padding to the overall min/max for better visibility
    padding_factor_y = 0.1
    padding_factor_z = 0.1

    range_x = max_overall_x - min_overall_x
    range_y = max_overall_y - min_overall_y
    range_z = max_overall_z - min_overall_z

    ax.set_xlim(min_overall_x - range_x * padding_factor_x, max_overall_x + range_x * padding_factor_x)
    ax.set_ylim(min_overall_y - range_y * padding_factor_y, max_overall_y + range_y * padding_factor_y)
    # Ensure z-axis starts near ground and extends above max altitude
    ax.set_zlim(min(min_overall_z, 0.0) - 1, max(max_overall_z, target_altitude + 5) + 1) # type: ignore


    ax.set_title("Drone Swarm - Staggered Pattern Formation with Wind")
    ax.set_xlabel("X (meters)")
    ax.set_ylabel("Y (meters)")
    ax.set_zlabel("Z (Altitude, meters)") # type: ignore
    ax.legend()
    ax.grid(True)

    # Animation function to update drone positions in each frame
    def update(frame):
        coords = history[frame]
        xs = [x for x, y, z in coords]
        ys = [y for x, y, z in coords]
        zs = [z for x, y, z in coords]
        scat._offsets3d = (xs, ys, zs)
        return scat,

    # Create the animation
    ani = animation.FuncAnimation(fig, update, frames=len(history), interval=50, blit=False)
    plt.show()

# No direct calls to run_visualization here, as it will be called from main.py
