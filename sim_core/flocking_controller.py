# # sim_core/flocking_controller.py
# """
# Implements a Boids-inspired flocking algorithm with sensor-fusion and relative ranging for tight formations.
# """
# import math
# from typing import List, Tuple, Optional
# from sensor_utils import get_corrected_position, fuse_gps_imu, get_range

# class FlockingController:
#     def __init__(self,
#                  perception_radius: float = 5.0,
#                  max_speed: float = 2.0,
#                  max_force: float = 0.05,
#                  weight_sep: float = 2.0,
#                  weight_align: float = 1.0,
#                  weight_cohesion: float = 1.0,
#                  weight_target: float = 0.5,
#                  kp_target: float = 0.1,
#                  nominal_spacing: float = 5.0):
#         self.perception_radius = perception_radius
#         self.max_speed = max_speed
#         self.max_force = max_force
#         self.weight_sep = weight_sep
#         self.weight_align = weight_align
#         self.weight_cohesion = weight_cohesion
#         self.weight_target = weight_target
#         self.kp_target = kp_target
#         self.nominal_spacing = nominal_spacing

#     def compute(self,
#                 raw_positions: List[Tuple[float, float, float]],
#                 imu_deltas: List[Tuple[float, float, float]],
#                 velocities: List[Tuple[float, float, float]],
#                 targets: Optional[List[Tuple[float, float, float]]] = None,
#                 use_fusion: bool = True
#                ) -> List[Tuple[float, float, float]]:
#         """
#         Compute acceleration vectors using:
#          - Sensor fusion (GPS+IMU)
#          - Separation (with UWB ranging)
#          - Alignment
#          - Cohesion
#          - Target attraction
#         """
#         # 1. Sensor fusion corrections
#         positions = []
#         for i, raw in enumerate(raw_positions):
#             pos = raw
#             if use_fusion:
#                 pos = fuse_gps_imu(raw, imu_deltas[i])
#             positions.append(pos)

#         n = len(positions)
#         accelerations: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * n
#         for i in range(n):
#             pos_i = positions[i]
#             v_i = velocities[i]
#             sep = [0.0, 0.0, 0.0]
#             align = [0.0, 0.0, 0.0]
#             coh = [0.0, 0.0, 0.0]
#             total = 0
#             # Neighborhood interactions
#             for j in range(n):
#                 if i == j: continue
#                 # alignment & cohesion as usual
#                 dx = positions[j][0] - pos_i[0]
#                 dy = positions[j][1] - pos_i[1]
#                 dz = positions[j][2] - pos_i[2]
#                 dist = math.sqrt(dx*dx + dy*dy + dz*dz)
#                 if dist < self.perception_radius and dist > 0:
#                     # separation enhanced by UWB ranging
#                     measured = get_range(pos_i, positions[j])
#                     # if too close compared to nominal spacing
#                     if measured < self.nominal_spacing:
#                         err = self.nominal_spacing - measured
#                         ux, uy, uz = dx/dist, dy/dist, dz/dist
#                         sep[0] -= ux * err
#                         sep[1] -= uy * err
#                         sep[2] -= uz * err
#                     # standard cohesion & alignment
#                     align[0] += velocities[j][0]
#                     align[1] += velocities[j][1]
#                     align[2] += velocities[j][2]
#                     coh[0] += positions[j][0]
#                     coh[1] += positions[j][1]
#                     coh[2] += positions[j][2]
#                     total += 1
#             # aggregate forces
#             ax = ay = az = 0.0
#             if total > 0:
#                 # finalize alignment
#                 align = [c/total for c in align]
#                 mag = math.sqrt(sum(c*c for c in align)) or 1.0
#                 align = [c/mag * self.max_force for c in align]
#                 # finalize cohesion
#                 coh = [coh[k]/total - pos_i[k] for k in range(3)]
#                 mag = math.sqrt(sum(c*c for c in coh)) or 1.0
#                 coh = [c/mag * self.max_force for c in coh]
#                 # finalize separation
#                 mag = math.sqrt(sum(c*c for c in sep)) or 1.0
#                 sep = [c/mag * self.max_force for c in sep]
#                 # weighted sum
#                 ax = self.weight_sep * sep[0] + self.weight_align * align[0] + self.weight_cohesion * coh[0]
#                 ay = self.weight_sep * sep[1] + self.weight_align * align[1] + self.weight_cohesion * coh[1]
#                 az = self.weight_sep * sep[2] + self.weight_align * align[2] + self.weight_cohesion * coh[2]
#             # target attraction
#             if targets:
#                 tx, ty, tz = targets[i]
#                 dx = tx - pos_i[0]
#                 dy = ty - pos_i[1]
#                 dz = tz - pos_i[2]
#                 dist = math.sqrt(dx*dx + dy*dy + dz*dz)
#                 if dist > 0:
#                     desired = [dx/dist * self.max_speed,
#                                dy/dist * self.max_speed,
#                                dz/dist * self.max_speed]
#                     targ_acc = [self.kp_target * (desired[k] - v_i[k]) for k in range(3)]
#                     ax += self.weight_target * targ_acc[0]
#                     ay += self.weight_target * targ_acc[1]
#                     az += self.weight_target * targ_acc[2]
#             # limit
#             mag = math.sqrt(ax*ax + ay*ay + az*az)
#             if mag > self.max_force:
#                 ax, ay, az = [c/mag * self.max_force for c in (ax, ay, az)]
#             accelerations[i] = (ax, ay, az)
#         return accelerations


# approach 2

# sim_core/flocking_controller.py
# """
# Implements a Boids-inspired flocking algorithm with sensor-fusion and relative ranging for tight formations.
# Now properly manages individual drone fused position history.
# """
# import math
# from typing import List, Tuple, Optional
# from sensor_utils import get_corrected_position, fuse_gps_imu, get_range

# class FlockingController:
#     def __init__(self,
#                  perception_radius: float = 5.0,
#                  max_speed: float = 2.0,
#                  max_force: float = 0.05,
#                  weight_sep: float = 2.0,
#                  weight_align: float = 1.0,
#                  weight_cohesion: float = 1.0,
#                  weight_target: float = 0.5,
#                  kp_target: float = 0.1,
#                  nominal_spacing: float = 5.0):
#         self.perception_radius = perception_radius
#         self.max_speed = max_speed
#         self.max_force = max_force
#         self.weight_sep = weight_sep
#         self.weight_align = weight_align
#         self.weight_cohesion = weight_cohesion
#         self.weight_target = weight_target
#         self.kp_target = kp_target
#         self.nominal_spacing = nominal_spacing

#         # Initialize a list to store the previously fused position for each drone
#         # This fixes the global variable bug from sensor_utils.py
#         self.prev_fused_positions: List[Tuple[float, float, float]] = []

#     def compute(self,
#                 raw_positions: List[Tuple[float, float, float]],
#                 imu_deltas: List[Tuple[float, float, float]],
#                 velocities: List[Tuple[float, float, float]],
#                 targets: Optional[List[Tuple[float, float, float]]] = None,
#                 use_fusion: bool = True
#                ) -> List[Tuple[float, float, float]]:
#         """
#         Compute acceleration vectors using:
#           - Sensor fusion (GPS+IMU) - now correctly managed per drone
#           - Separation (with UWB ranging)
#           - Alignment
#           - Cohesion
#           - Target attraction
#         """
#         n = len(raw_positions)

#         # Initialize prev_fused_positions for new drones or on first run
#         if not self.prev_fused_positions or len(self.prev_fused_positions) != n:
#             self.prev_fused_positions = [(0.0, 0.0, 0.0)] * n

#         # 1. Sensor fusion corrections (now per drone, state managed by controller)
#         current_fused_positions = []
#         for i, raw_pos_i in enumerate(raw_positions):
#             fused_pos_i = raw_pos_i # Start with raw as base
#             if use_fusion:
#                 # Use the drone's specific previous fused position for the filter
#                 fused_pos_i = fuse_gps_imu(raw_pos_i, imu_deltas[i], self.prev_fused_positions[i])
#             current_fused_positions.append(fused_pos_i)
        
#         # Update the controller's stored previous fused positions for the next timestep
#         self.prev_fused_positions = current_fused_positions[:] # Make a copy

#         # Use the newly computed current_fused_positions for flocking calculations
#         positions = current_fused_positions

#         accelerations: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * n
#         for i in range(n):
#             pos_i = positions[i]
#             v_i = velocities[i]
#             sep = [0.0, 0.0, 0.0]
#             align = [0.0, 0.0, 0.0]
#             coh = [0.0, 0.0, 0.0]
#             total = 0
#             # Neighborhood interactions
#             for j in range(n):
#                 if i == j: continue
                
#                 dx = positions[j][0] - pos_i[0]
#                 dy = positions[j][1] - pos_i[1]
#                 dz = positions[j][2] - pos_i[2]
#                 dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
#                 if dist < self.perception_radius and dist > 0:
#                     # separation enhanced by UWB ranging
#                     measured = get_range(pos_i, positions[j]) # Use fused positions for ranging
#                     # if too close compared to nominal spacing
#                     if measured < self.nominal_spacing:
#                         err = self.nominal_spacing - measured
#                         # Normalize vector from j to i (away from j)
#                         ux, uy, uz = dx/dist, dy/dist, dz/dist 
#                         sep[0] -= ux * err # Push away
#                         sep[1] -= uy * err
#                         sep[2] -= uz * err
                    
#                     # standard cohesion & alignment
#                     align[0] += velocities[j][0]
#                     align[1] += velocities[j][1]
#                     align[2] += velocities[j][2]
                    
#                     coh[0] += positions[j][0]
#                     coh[1] += positions[j][1]
#                     coh[2] += positions[j][2]
#                     total += 1
            
#             # aggregate forces
#             ax = ay = az = 0.0
#             if total > 0:
#                 # finalize alignment
#                 align = [c/total for c in align]
#                 mag = math.sqrt(sum(c*c for c in align)) or 1.0
#                 align = [c/mag * self.max_force for c in align]
                
#                 # finalize cohesion
#                 coh = [coh[k]/total - pos_i[k] for k in range(3)] # Vector to center of mass
#                 mag = math.sqrt(sum(c*c for c in coh)) or 1.0
#                 coh = [c/mag * self.max_force for c in coh]
                
#                 # finalize separation
#                 mag = math.sqrt(sum(c*c for c in sep)) or 1.0
#                 sep = [c/mag * self.max_force for c in sep]
                
#                 # weighted sum of flocking forces
#                 ax = self.weight_sep * sep[0] + self.weight_align * align[0] + self.weight_cohesion * coh[0]
#                 ay = self.weight_sep * sep[1] + self.weight_align * align[1] + self.weight_cohesion * coh[1]
#                 az = self.weight_sep * sep[2] + self.weight_align * align[2] + self.weight_cohesion * coh[2]
            
#             # target attraction (steering towards target velocity)
#             if targets:
#                 tx, ty, tz = targets[i]
#                 dx = tx - pos_i[0]
#                 dy = ty - pos_i[1]
#                 dz = tz - pos_i[2]
#                 dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
#                 if dist > 0:
#                     desired_vel = [dx/dist * self.max_speed, # Desired velocity towards target
#                                    dy/dist * self.max_speed,
#                                    dz/dist * self.max_speed]
                    
#                     # Proportional control on velocity error
#                     targ_acc = [self.kp_target * (desired_vel[k] - v_i[k]) for k in range(3)]
                    
#                     ax += self.weight_target * targ_acc[0]
#                     ay += self.weight_target * targ_acc[1]
#                     az += self.weight_target * targ_acc[2]
            
#             # Limit the total acceleration/force
#             mag = math.sqrt(ax*ax + ay*ay + az*az)
#             if mag > self.max_force:
#                 ax, ay, az = [c/mag * self.max_force for c in (ax, ay, az)]
            
#             accelerations[i] = (ax, ay, az)
        
#         return accelerations





# approach 3

# # sim_core/flocking_controller.py
# """
# Implements a Boids-inspired flocking algorithm with sensor-fusion and relative ranging for tight formations.
# Now properly manages individual drone fused position history.
# Includes dynamic target attraction for smoother and more robust hovering, addressing drift.
# """
# import math
# from typing import List, Tuple, Optional
# from sensor_utils import get_corrected_position, fuse_gps_imu, get_range

# class FlockingController:
#     def __init__(self,
#                  perception_radius: float = 5.0,
#                  max_speed: float = 2.0,
#                  max_force: float = 0.05,
#                  weight_sep: float = 2.0,
#                  weight_align: float = 1.0,
#                  weight_cohesion: float = 1.0,
#                  weight_target: float = 0.5,
#                  kp_target: float = 0.1,
#                  nominal_spacing: float = 5.0,
#                  hover_slowing_radius: float = 5.0 # New parameter for dynamic target attraction
#                 ):
#         self.perception_radius = perception_radius
#         self.max_speed = max_speed
#         self.max_force = max_force
#         self.weight_sep = weight_sep
#         self.weight_align = weight_align
#         self.weight_cohesion = weight_cohesion
#         self.weight_target = weight_target
#         self.kp_target = kp_target
#         self.nominal_spacing = nominal_spacing
#         self.hover_slowing_radius = hover_slowing_radius

#         # Initialize a list to store the previously fused position for each drone
#         self.prev_fused_positions: List[Tuple[float, float, float]] = []

#     def compute(self,
#                 raw_positions: List[Tuple[float, float, float]],
#                 imu_deltas: List[Tuple[float, float, float]],
#                 velocities: List[Tuple[float, float, float]],
#                 targets: Optional[List[Tuple[float, float, float]]] = None,
#                 use_fusion: bool = True
#                ) -> List[Tuple[float, float, float]]:
#         """
#         Compute acceleration vectors using:
#           - Sensor fusion (GPS+IMU) - now correctly managed per drone
#           - Separation (with UWB ranging)
#           - Alignment
#           - Cohesion
#           - Target attraction (dynamically adjusted for hovering)
#         """
#         n = len(raw_positions)

#         # Initialize prev_fused_positions for new drones or on first run
#         if not self.prev_fused_positions or len(self.prev_fused_positions) != n:
#             self.prev_fused_positions = [(0.0, 0.0, 0.0)] * n

#         # 1. Sensor fusion corrections (now per drone, state managed by controller)
#         current_fused_positions = []
#         for i, raw_pos_i in enumerate(raw_positions):
#             fused_pos_i = raw_pos_i # Start with raw as base
#             if use_fusion:
#                 # Use the drone's specific previous fused position for the filter
#                 fused_pos_i = fuse_gps_imu(raw_pos_i, imu_deltas[i], self.prev_fused_positions[i])
#             current_fused_positions.append(fused_pos_i)
        
#         # Update the controller's stored previous fused positions for the next timestep
#         self.prev_fused_positions = current_fused_positions[:] # Make a copy

#         # Use the newly computed current_fused_positions for flocking calculations
#         positions = current_fused_positions

#         accelerations: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * n
#         # Define a small epsilon to prevent division by zero for distances
#         EPSILON = 1e-6 

#         for i in range(n):
#             pos_i = positions[i]
#             v_i = velocities[i]
#             sep = [0.0, 0.0, 0.0]
#             align = [0.0, 0.0, 0.0]
#             coh = [0.0, 0.0, 0.0]
#             total = 0
#             # Neighborhood interactions
#             for j in range(n):
#                 if i == j: continue
                
#                 dx = positions[j][0] - pos_i[0]
#                 dy = positions[j][1] - pos_i[1]
#                 dz = positions[j][2] - pos_i[2]
#                 dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
#                 if dist < self.perception_radius and dist > EPSILON: # Use EPSILON here
#                     # separation enhanced by UWB ranging
#                     measured = get_range(pos_i, positions[j]) # Use fused positions for ranging
#                     # if too close compared to nominal spacing
#                     if measured < self.nominal_spacing:
#                         err = self.nominal_spacing - measured
#                         # Normalize vector from j to i (away from j)
#                         ux, uy, uz = dx/dist, dy/dist, dz/dist 
#                         sep[0] -= ux * err # Push away
#                         sep[1] -= uy * err
#                         sep[2] -= uz * err
                    
#                     # standard cohesion & alignment
#                     align[0] += velocities[j][0]
#                     align[1] += velocities[j][1]
#                     align[2] += velocities[j][2]
                    
#                     coh[0] += positions[j][0]
#                     coh[1] += positions[j][1]
#                     coh[2] += positions[j][2]
#                     total += 1
            
#             # aggregate forces
#             ax = ay = az = 0.0
#             if total > 0:
#                 # finalize alignment
#                 align = [c/total for c in align]
#                 mag = math.sqrt(sum(c*c for c in align)) or 1.0
#                 align = [c/mag * self.max_force for c in align]
                
#                 # finalize cohesion
#                 coh = [coh[k]/total - pos_i[k] for k in range(3)] # Vector to center of mass
#                 mag = math.sqrt(sum(c*c for c in coh)) or 1.0
#                 coh = [c/mag * self.max_force for c in coh]
                
#                 # finalize separation
#                 mag = math.sqrt(sum(c*c for c in sep)) or 1.0
#                 sep = [c/mag * self.max_force for c in sep]
                
#                 # weighted sum of flocking forces
#                 ax = self.weight_sep * sep[0] + self.weight_align * align[0] + self.weight_cohesion * coh[0]
#                 ay = self.weight_sep * sep[1] + self.weight_align * align[1] + self.weight_cohesion * coh[1]
#                 az = self.weight_sep * sep[2] + self.weight_align * align[2] + self.weight_cohesion * coh[2]
            
#             # target attraction (steering towards target velocity)
#             if targets:
#                 tx, ty, tz = targets[i]
#                 dx = tx - pos_i[0]
#                 dy = ty - pos_i[1]
#                 dz = tz - pos_i[2]
#                 dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
#                 if dist > EPSILON: # Use EPSILON here
#                     effective_kp_target = self.kp_target
#                     min_kp_factor_val = 0.05 # Minimum kp_target factor when very close
                    
#                     # Ensure hover_slowing_radius is always greater than nominal_spacing for a valid slowing zone
#                     # If they are too close, slightly adjust hover_slowing_radius
#                     adjusted_hover_slowing_radius = max(self.hover_slowing_radius, self.nominal_spacing + 1.0) # Ensure at least 1m difference

#                     if dist <= self.nominal_spacing:
#                         # Very close to nominal spacing, apply minimum attraction
#                         effective_kp_target = self.kp_target * min_kp_factor_val
#                     elif dist < adjusted_hover_slowing_radius:
#                         # Inside the slowing zone, gradually reduce attraction
#                         # Scale linearly from 1.0 at adjusted_hover_slowing_radius to min_kp_factor_val at nominal_spacing
#                         scale_progress = (dist - self.nominal_spacing) / (adjusted_hover_slowing_radius - self.nominal_spacing)
#                         scale_progress = max(0.0, min(1.0, scale_progress)) # Clamp to [0, 1]
#                         effective_kp_target = self.kp_target * (min_kp_factor_val + (1.0 - min_kp_factor_val) * scale_progress)
#                     # Else (dist >= adjusted_hover_slowing_radius), effective_kp_target remains self.kp_target (full strength)

#                     # Add damping to prevent overshooting and oscillations when near target
#                     # Damping strength increases significantly as drone approaches target
#                     damping_coeff = 0.0
#                     # Apply damping when within a larger radius around the target
#                     if dist < adjusted_hover_slowing_radius * 1.5: 
#                         # Scale damping from 0 (at 1.5*hover_slowing_radius) to max_damping_coeff (at nominal_spacing)
#                         max_damping_coeff = 5.0 # Max damping strength
                        
#                         # Calculate a scaling factor: closer = higher damping
#                         damping_scale = 1.0 - (dist - self.nominal_spacing) / (adjusted_hover_slowing_radius * 1.5 - self.nominal_spacing)
#                         damping_scale = max(0.0, min(1.0, damping_scale)) # Clamp to [0, 1]
#                         damping_coeff = max_damping_coeff * damping_scale
                        
#                         ax -= v_i[0] * damping_coeff
#                         ay -= v_i[1] * damping_coeff
#                         az -= v_i[2] * damping_coeff

#                     desired_vel = [dx/dist * self.max_speed, # Desired velocity towards target
#                                    dy/dist * self.max_speed,
#                                    dz/dist * self.max_speed]
                    
#                     # Proportional control on velocity error using effective_kp_target
#                     targ_acc = [effective_kp_target * (desired_vel[k] - v_i[k]) for k in range(3)]
                    
#                     ax += self.weight_target * targ_acc[0]
#                     ay += self.weight_target * targ_acc[1]
#                     az += self.weight_target * targ_acc[2]
            
#             # Limit the total acceleration/force
#             mag = math.sqrt(ax*ax + ay*ay + az*az)
#             if mag > self.max_force:
#                 ax, ay, az = [c/mag * self.max_force for c in (ax, ay, az)]
            
#             accelerations[i] = (ax, ay, az)
        
#         return accelerations






# approach 4





# sim_core/flocking_controller.py
"""
Implements a Boids-inspired flocking algorithm with sensor-fusion and relative ranging for tight formations.
Now properly manages individual drone fused position history.
Includes dynamic target attraction for smoother and more robust hovering, addressing drift.
"""
import math
from typing import List, Tuple, Optional
from sensor_utils import get_corrected_position, fuse_gps_imu, get_range

class FlockingController:
    def __init__(self,
                 perception_radius: float = 5.0,
                 max_speed: float = 2.0,
                 max_force: float = 0.05,
                 weight_sep: float = 2.0,
                 weight_align: float = 1.0,
                 weight_cohesion: float = 1.0,
                 weight_target: float = 0.5,
                 kp_target: float = 0.1,
                 nominal_spacing: float = 5.0,
                 hover_slowing_radius: float = 5.0 # New parameter for dynamic target attraction
                ):
        self.perception_radius = perception_radius
        self.max_speed = max_speed
        self.max_force = max_force
        self.weight_sep = weight_sep
        self.weight_align = weight_align
        self.weight_cohesion = weight_cohesion
        self.weight_target = weight_target
        self.kp_target = kp_target
        self.nominal_spacing = nominal_spacing
        self.hover_slowing_radius = hover_slowing_radius

        # Initialize a list to store the previously fused position for each drone
        self.prev_fused_positions: List[Tuple[float, float, float]] = []

    def compute(self,
                raw_positions: List[Tuple[float, float, float]],
                imu_deltas: List[Tuple[float, float, float]],
                velocities: List[Tuple[float, float, float]],
                targets: Optional[List[Tuple[float, float, float]]] = None,
                use_fusion: bool = True
               ) -> List[Tuple[float, float, float]]:
        """
        Compute acceleration vectors using:
          - Sensor fusion (GPS+IMU) - now correctly managed per drone
          - Separation (with UWB ranging)
          - Alignment
          - Cohesion
          - Target attraction (dynamically adjusted for hovering)
        """
        n = len(raw_positions)

        # Initialize prev_fused_positions for new drones or on first run
        if not self.prev_fused_positions or len(self.prev_fused_positions) != n:
            self.prev_fused_positions = [(0.0, 0.0, 0.0)] * n

        # 1. Sensor fusion corrections (now per drone, state managed by controller)
        current_fused_positions = []
        for i, raw_pos_i in enumerate(raw_positions):
            fused_pos_i = raw_pos_i # Start with raw as base
            if use_fusion:
                # Use the drone's specific previous fused position for the filter
                fused_pos_i = fuse_gps_imu(raw_pos_i, imu_deltas[i], self.prev_fused_positions[i])
            current_fused_positions.append(fused_pos_i)
        
        # Update the controller's stored previous fused positions for the next timestep
        self.prev_fused_positions = current_fused_positions[:] # Make a copy

        # Use the newly computed current_fused_positions for flocking calculations
        positions = current_fused_positions

        accelerations: List[Tuple[float, float, float]] = [(0.0, 0.0, 0.0)] * n
        # Define a small epsilon to prevent division by zero for distances
        EPSILON = 1e-6 

        for i in range(n):
            pos_i = positions[i]
            v_i = velocities[i]
            sep = [0.0, 0.0, 0.0]
            align = [0.0, 0.0, 0.0]
            coh = [0.0, 0.0, 0.0]
            total = 0
            # Neighborhood interactions
            for j in range(n):
                if i == j: continue
                
                dx = positions[j][0] - pos_i[0]
                dy = positions[j][1] - pos_i[1]
                dz = positions[j][2] - pos_i[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if dist < self.perception_radius and dist > EPSILON: # Use EPSILON here
                    # separation enhanced by UWB ranging
                    measured = get_range(pos_i, positions[j]) # Use fused positions for ranging
                    # if too close compared to nominal spacing
                    if measured < self.nominal_spacing:
                        err = self.nominal_spacing - measured
                        # Normalize vector from j to i (away from j)
                        ux, uy, uz = dx/dist, dy/dist, dz/dist 
                        sep[0] -= ux * err # Push away
                        sep[1] -= uy * err
                        sep[2] -= uz * err
                    
                    # standard cohesion & alignment
                    align[0] += velocities[j][0]
                    align[1] += velocities[j][1]
                    align[2] += velocities[j][2]
                    
                    coh[0] += positions[j][0]
                    coh[1] += positions[j][1]
                    coh[2] += positions[j][2]
                    total += 1
            
            # aggregate forces
            ax = ay = az = 0.0
            if total > 0:
                # finalize alignment
                align = [c/total for c in align]
                mag = math.sqrt(sum(c*c for c in align)) or 1.0
                align = [c/mag * self.max_force for c in align]
                
                # finalize cohesion
                coh = [coh[k]/total - pos_i[k] for k in range(3)] # Vector to center of mass
                mag = math.sqrt(sum(c*c for c in coh)) or 1.0
                coh = [c/mag * self.max_force for c in coh]
                
                # finalize separation
                mag = math.sqrt(sum(c*c for c in sep)) or 1.0
                sep = [c/mag * self.max_force for c in sep]
                
                # weighted sum of flocking forces
                ax = self.weight_sep * sep[0] + self.weight_align * align[0] + self.weight_cohesion * coh[0]
                ay = self.weight_sep * sep[1] + self.weight_align * align[1] + self.weight_cohesion * coh[1]
                az = self.weight_sep * sep[2] + self.weight_align * align[2] + self.weight_cohesion * coh[2]
            
            # target attraction (steering towards target velocity)
            if targets:
                tx, ty, tz = targets[i]
                dx = tx - pos_i[0]
                dy = ty - pos_i[1]
                dz = tz - pos_i[2]
                dist = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if dist > EPSILON: # Use EPSILON here
                    effective_kp_target = self.kp_target
                    min_kp_factor_val = 0.05 # Minimum kp_target factor when very close
                    
                    # Ensure hover_slowing_radius is always greater than nominal_spacing for a valid slowing zone
                    # If they are too close, slightly adjust hover_slowing_radius
                    # Adding a safety margin to nominal_spacing for the calculation
                    slowing_zone_start = self.hover_slowing_radius
                    slowing_zone_end = self.nominal_spacing # Target attraction very low once inside this
                    
                    # To prevent division by zero and ensure a valid range
                    slowing_zone_diff = slowing_zone_start - slowing_zone_end
                    if slowing_zone_diff < 1.0: # Ensure a minimum difference of 1 meter
                        slowing_zone_diff = 1.0
                        slowing_zone_start = slowing_zone_end + 1.0 # Adjust start if too small
                    
                    if dist <= slowing_zone_end:
                        # Very close to nominal spacing, apply minimum attraction
                        effective_kp_target = self.kp_target * min_kp_factor_val
                    elif dist < slowing_zone_start:
                        # Inside the slowing zone, gradually reduce attraction
                        # Scale from (1.0 at slowing_zone_start) down to (min_kp_factor_val at slowing_zone_end)
                        scale_progress = (dist - slowing_zone_end) / slowing_zone_diff
                        scale_progress = max(0.0, min(1.0, scale_progress)) # Clamp to [0, 1]
                        
                        effective_kp_target = self.kp_target * (min_kp_factor_val + (1.0 - min_kp_factor_val) * scale_progress)
                    # Else (dist >= slowing_zone_start), effective_kp_target remains self.kp_target (full strength)


                    # Add damping to prevent overshooting and oscillations when near target
                    # Damping strength increases significantly as drone approaches target
                    damping_coeff = 0.0
                    damping_active_radius = max(self.hover_slowing_radius * 2.0, 30.0) # Active damping over a larger radius
                    
                    if dist < damping_active_radius: 
                        # Scale damping from 0 (at damping_active_radius) to max_damping_coeff (at target)
                        max_damping_coeff = 10.0 # Increased max damping strength for better stability
                        
                        # Calculate a scaling factor: closer = higher damping
                        # Ensure denominator is not zero or too small
                        damping_range = damping_active_radius - self.nominal_spacing
                        if damping_range < 0.1: damping_range = 0.1 # Safety check
                        
                        damping_scale = 1.0 - (dist - self.nominal_spacing) / damping_range
                        damping_scale = max(0.0, min(1.0, damping_scale)) # Clamp to [0, 1]
                        
                        damping_coeff = max_damping_coeff * damping_scale
                        
                        ax -= v_i[0] * damping_coeff
                        ay -= v_i[1] * damping_coeff
                        az -= v_i[2] * damping_coeff
                    
                    # Ensure a minimal target pull even when very close to counteract sensor noise/drift
                    # This is applied *after* damping calculation to ensure stability
                    min_pull_strength = self.max_force * 0.01 # A very small constant pull
                    if effective_kp_target < min_pull_strength:
                        effective_kp_target = min_pull_strength

                    desired_vel = [dx/dist * self.max_speed, # Desired velocity towards target
                                   dy/dist * self.max_speed,
                                   dz/dist * self.max_speed]
                    
                    # Proportional control on velocity error using effective_kp_target
                    targ_acc = [effective_kp_target * (desired_vel[k] - v_i[k]) for k in range(3)]
                    
                    ax += self.weight_target * targ_acc[0]
                    ay += self.weight_target * targ_acc[1]
                    az += self.weight_target * targ_acc[2]
            
            # Limit the total acceleration/force
            mag = math.sqrt(ax*ax + ay*ay + az*az)
            if mag > self.max_force:
                ax, ay, az = [c/mag * self.max_force for c in (ax, ay, az)]
            
            accelerations[i] = (ax, ay, az)
        
        return accelerations

