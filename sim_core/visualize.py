# Second try

# sim_core/visualize.py
# """
# Visualize drone movement forming a rectangular pattern in 3D using Matplotlib animation.
# Now correctly uses the FormationPlanner.rectangle and enables full flocking behaviors.
# Adjusted parameters for stable flocking and target hovering.
# Drones now start bundled from a low altitude.
# The flocking controller handles soft target attraction for hovering, no rigid stopping.
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
# NUM_DRONES = 16 # Example for a 4x4 or 8x2 distribution on a perimeter, or 4x4 internal grid
# TIMESTEPS = 2000 # Increased steps significantly to allow for long-term stability observation
# DT = 0.1

# # Rectangle formation parameters
# # These define the overall dimensions of the rectangular pattern
# RECTANGLE_WIDTH = 40.0  # Total width of the rectangle
# RECTANGLE_HEIGHT = 20.0 # Total height of the rectangle
# ALTITUDE = 12.0         # Flight height above ground

# # Initial state for drones: bundled together near the origin at a low altitude
# positions = [
#     (random.uniform(-2.0, 2.0),  # Small random spread around 0 for X
#      random.uniform(-2.0, 2.0),  # Small random spread around 0 for Y
#      1.0)                       # Start near the ground (low Z)
#     for _ in range(NUM_DRONES)
# ]
# velocities = [(0.0, 0.0, 0.0)] * NUM_DRONES
# imu_deltas = [(0.0, 0.0, 0.0)] * NUM_DRONES # Will be updated in loop

# # Generate rectangular formation targets using the FormationPlanner
# targets = FormationPlanner.rectangle(NUM_DRONES, RECTANGLE_WIDTH, RECTANGLE_HEIGHT, ALTITUDE)

# # Flocking controller configured for stable rectangular formation and hovering
# flock = FlockingController(
#     perception_radius=max(RECTANGLE_WIDTH, RECTANGLE_HEIGHT) * 0.75, # Perceive most of the flock
#     max_speed=5.0,
#     max_force=0.2,      # Increased max force slightly to give more authority
#     weight_sep=3.5,     # Increased separation weight even more for stronger collision avoidance
#     weight_align=1.0,   # Enable alignment
#     weight_cohesion=1.0,# Enable cohesion
#     weight_target=4.0,  # Stronger target attraction to pull them faster to the general area
#     kp_target=0.4,      # Increased base proportional gain for target steering
#     nominal_spacing=5.0, # Desired spacing when flocking
#     hover_slowing_radius=20.0 # Drones start slowing target attraction when within 20m of target
# )

# print(f"Visualization simulation started for {NUM_DRONES} drones.")
# print(f"Targeting a {RECTANGLE_WIDTH}m x {RECTANGLE_HEIGHT}m rectangle at {ALTITUDE}m altitude.")
# print("Generated target positions:")
# for i, t in enumerate(targets):
#     print(f"  Drone {i} Target: ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

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

# # Set dynamic axes limits based on rectangle dimensions to ensure all drones are visible
# padding_x = RECTANGLE_WIDTH * 0.15 # Add some padding around the rectangle
# padding_y = RECTANGLE_HEIGHT * 0.15
# ax.set_xlim(-(RECTANGLE_WIDTH/2 + padding_x), (RECTANGLE_WIDTH/2 + padding_x))
# ax.set_ylim(-(RECTANGLE_HEIGHT/2 + padding_y), (RECTANGLE_HEIGHT/2 + padding_y)) # Center Y for rectangle
# ax.set_zlim(ALTITUDE - 5, ALTITUDE + 5) # Enough room around the altitude

# ax.set_title("Drone Swarm - Rectangular Formation")
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






# approach 3




# sim_core/visualize.py
# """
# Visualize drone movement forming a staggered pattern in 3D using Matplotlib animation.
# Now uses the precise staggered pattern logic from FormationPlanner.
# Adjusted parameters for stable flocking and hovering around targets.
# Drones now start bundled from a low altitude.
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
# TIMESTEPS = 2500 # Increased steps significantly for long-term stability and initial movement
# DT = 0.1

# # Staggered pattern formation parameters
# HORIZONTAL_SPACING = 8.0 # Horizontal distance between drones in a row
# VERTICAL_SPACING = 6.0   # Vertical distance between the two rows
# ALTITUDE = 12.0          # Flight height above ground

# # Initial state for drones: bundled together near the origin at a low altitude
# positions = [
#     (random.uniform(-2.0, 2.0),  # Small random spread around 0 for X
#      random.uniform(-2.0, 2.0),  # Small random spread around 0 for Y
#      1.0)                       # Start near the ground (low Z)
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
# flock = FlockingController(
#     perception_radius=max(max_x_target - min_x_target, max_y_target - min_y_target) * 0.75 + 10.0, # Adjust perception
#     max_speed=5.0,
#     max_force=0.2,      # Max force applied by controller
#     weight_sep=3.5,     # Increased separation weight for stronger collision avoidance
#     weight_align=1.0,   # Enable alignment
#     weight_cohesion=1.0,# Enable cohesion
#     weight_target=4.0,  # Stronger target attraction to pull them faster to the general area
#     kp_target=0.4,      # Increased base proportional gain for target steering
#     nominal_spacing=5.0, # Desired spacing when flocking
#     hover_slowing_radius=15.0 # Drones start slowing target attraction when within 15m of target
# )

# print(f"Visualization simulation started for {NUM_DRONES} drones.")
# print(f"Targeting a staggered pattern with horizontal spacing {HORIZONTAL_SPACING}m and vertical spacing {VERTICAL_SPACING}m at {ALTITUDE}m altitude.")
# print("Generated target positions:")
# for i, t in enumerate(targets):
#     print(f"  Drone {i} Target: ({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

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

# # Set dynamic axes limits based on calculated target pattern bounds
# padding_x = (max_x_target - min_x_target) * 0.20 # Add some padding around the pattern
# padding_y = (max_y_target - min_y_target) * 0.20
# ax.set_xlim(min_x_target - padding_x, max_x_target + padding_x)
# ax.set_ylim(min_y_target - padding_y, max_y_target + padding_y) 
# ax.set_zlim(ALTITUDE - 5, ALTITUDE + 5) # Enough room around the altitude

# ax.set_title("Drone Swarm - Staggered Pattern Formation")
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




# approach 4




# sim_core/visualize.py
"""
Visualize drone movement forming a staggered pattern in 3D using Matplotlib animation.
Now uses the precise staggered pattern logic from FormationPlanner.
Adjusted parameters for stable flocking and hovering around targets.
Drones now start from a wider range of initial positions to test robust navigation.
"""
import matplotlib.pyplot as plt
import matplotlib.animation as animation
from mpl_toolkits.mplot3d import Axes3D
from flocking_controller import FlockingController # Import the updated FlockingController
from formation_planner import FormationPlanner # Import the updated FormationPlanner
import random
import json
import math
from sensor_utils import get_range # Only need get_range explicitly here if not for debug prints

# Simulation constants
NUM_DRONES = 7 # Example for the 4+3 staggered pattern from the image
TIMESTEPS = 3500 # Increased steps significantly for long-term stability and initial movement
DT = 0.1

# Staggered pattern formation parameters
HORIZONTAL_SPACING = 9.0 # Horizontal distance between drones in a row
VERTICAL_SPACING = 4.0   # Vertical distance between the two rows
ALTITUDE = 12.0          # Flight height above ground

# Initial state for drones: scattered over a larger area, starting from low Z
positions = [
    (random.uniform(-20.0, 20.0),  # Wider random spread for X
     random.uniform(-20.0, 20.0),  # Wider random spread for Y
     1.0)                        # All drones start from 1.0m altitude (ground)
    for _ in range(NUM_DRONES)
]
velocities = [(0.0, 0.0, 0.0)] * NUM_DRONES
imu_deltas = [(0.0, 0.0, 0.0)] * NUM_DRONES # Will be updated in loop

# Generate staggered pattern targets using the FormationPlanner
targets = FormationPlanner.staggered_pattern(NUM_DRONES, HORIZONTAL_SPACING, VERTICAL_SPACING, ALTITUDE)

# Calculate the bounds of the target pattern for plot limits
min_x_target = min(t[0] for t in targets)
max_x_target = max(t[0] for t in targets)
min_y_target = min(t[1] for t in targets)
max_y_target = max(t[1] for t in targets)

# Flocking controller configured for stable formation and hovering
# Parameters adjusted for more robust navigation from arbitrary starting points
flock = FlockingController(
    perception_radius=max(max_x_target - min_x_target, max_y_target - min_y_target) * 0.75 + 20.0, # Increased perception further
    max_speed=9.0,     # Significantly increased max_speed for faster long-distance travel
    max_force=0.6,      # Significantly increased max_force for stronger initial acceleration
    weight_sep=5.0,     # Increased separation weight even more for stronger collision avoidance
    weight_align=1.0,   # Enable alignment
    weight_cohesion=1.0,# Enable cohesion
    weight_target=8.0,  # Even stronger target attraction to pull them from far away
    kp_target=0.8,      # Increased base proportional gain for target steering
    nominal_spacing=5.0, # Desired spacing when flocking
    hover_slowing_radius=25.0 # Drones start slowing target attraction when within 25m of target
)

print(f"Visualization simulation started for {NUM_DRONES} drones.")
print(f"Targeting a staggered pattern with horizontal spacing {HORIZONTAL_SPACING}m and vertical spacing {VERTICAL_SPACING}m at {ALTITUDE}m altitude.")
print("Generated target positions:")
for i, t in enumerate(targets):
    print(f"  Drone {i}: Pos=({t[0]:.2f}, {t[1]:.2f}, {t[2]:.2f})")

# History for animation and logging
history = []

for step in range(TIMESTEPS):
    # Simulate small IMU deltas (noise for dead reckoning)
    imu_deltas = [(
        random.gauss(0, 0.005), 
        random.gauss(0, 0.005), 
        random.gauss(0, 0.002)
    ) for _ in range(NUM_DRONES)]

    # Compute accelerations using the flocking controller
    accs = flock.compute(
        positions,
        imu_deltas,
        velocities,
        targets,
        use_fusion=True
    )

    new_vel = []
    new_pos = []
    for i in range(NUM_DRONES):
        vx, vy, vz = velocities[i]
        ax, ay, az = accs[i]

        # Euler integration for velocity update
        vx_new = vx + ax * DT
        vy_new = vy + ay * DT
        vz_new = vz + az * DT

        # Limit speed to the controller's max_speed
        speed = math.sqrt(vx_new**2 + vy_new**2 + vz_new**2)
        if speed > flock.max_speed:
            factor = flock.max_speed / speed
            vx_new *= factor
            vy_new *= factor
            vz_new *= factor
        
        # Euler integration for position update
        x, y, z = positions[i]
        x_new = x + vx_new * DT
        y_new = y + vy_new * DT
        z_new = z + vz_new * DT
        
        new_vel.append((vx_new, vy_new, vz_new))
        new_pos.append((x_new, y_new, z_new))
    
    velocities = new_vel
    positions = new_pos
    history.append([list(pos) for pos in positions]) # Store current positions for this timestep

    if step % 50 == 0 or step == TIMESTEPS - 1:
        print(f"\n--- Step {step} ---")
        for i, pos in enumerate(positions):
            target_pos = targets[i]
            dist_to_target = math.sqrt(
                (pos[0] - target_pos[0])**2 + 
                (pos[1] - target_pos[1])**2 + 
                (pos[2] - target_pos[2])**2
            )
            print(f"  Drone {i}: Pos=({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f}), Dist to Target={dist_to_target:.2f}m")

print("\nSimulation complete. Generating visualization...")

# Save log to file (optional, for external analysis)
with open("visualize_drone_log.json", "w") as f:
    json.dump(history, f)

# 3D Visualization setup
fig = plt.figure(figsize=(10, 8))
ax = fig.add_subplot(111, projection='3d')

# Drone scatter plot (will be updated in animation)
scat = ax.scatter([], [], [], s=80, c='blue', label='Drones')

# Target scatter plot (static, 'X' markers for clarity)
txs = [t[0] for t in targets]
tys = [t[1] for t in targets]
tzs = [t[2] for t in targets]
ax.scatter(txs, tys, tzs, s=100, c='red', marker='X', label='Targets')

# Frontal view: camera at y-positive looking toward negative y. Adjust 'azim' for different views.
ax.view_init(elev=20, azim=180) 

# Set dynamic axes limits based on calculated target pattern bounds and initial spread
initial_min_x = min(p[0] for p in history[0])
initial_max_x = max(p[0] for p in history[0])
initial_min_y = min(p[1] for p in history[0])
initial_max_y = max(p[1] for p in history[0])
initial_min_z = min(p[2] for p in history[0])
initial_max_z = max(p[2] for p in history[0])

# Combine initial and target bounds for comprehensive limits
overall_min_x = min(min_x_target, initial_min_x)
overall_max_x = max(max_x_target, initial_max_x)
overall_min_y = min(min_y_target, initial_min_y)
overall_max_y = max(max_y_target, initial_max_y)
# For Z, ensure it covers ground (1.0) up to max initial Z and target altitude + padding
overall_min_z = 1.0 # Drones start at 1m
overall_max_z = max(ALTITUDE + 5, initial_max_z + 5) # Pad initial max_z and target altitude

padding_factor_x = 0.3 # Increased padding for wider initial spread
padding_factor_y = 0.3
padding_factor_z = 0.3

ax.set_xlim(overall_min_x - (overall_max_x - overall_min_x) * padding_factor_x, overall_max_x + (overall_max_x - overall_min_x) * padding_factor_x)
ax.set_ylim(overall_min_y - (overall_max_y - overall_min_y) * padding_factor_y, overall_max_y + (overall_max_y - overall_min_y) * padding_factor_y)
ax.set_zlim(overall_min_z - 1, overall_max_z + 1) # Small padding around ground and max altitude


ax.set_title("Drone Swarm - Staggered Pattern Formation (Robust Navigation)")
ax.set_xlabel("X (meters)")
ax.set_ylabel("Y (meters)")
ax.set_zlabel("Z (Altitude, meters)")
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

# To display the animation, you need to call plt.show()
plt.show()



