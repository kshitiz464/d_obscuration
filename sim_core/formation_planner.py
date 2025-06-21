
# # base approach 1

# # sim_core/formation_planner.py
# """
# Defines various formation target generators: wall, ring, ellipse, hex-grid, and now rectangle.
# """
# import math
# from typing import List, Tuple

# class FormationPlanner:
#     @staticmethod
#     def wall(num: int, spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
#         # Centered wall along x-axis
#         start = -(num-1)/2 * spacing
#         return [(start + i*spacing, 0.0, altitude) for i in range(num)]

#     @staticmethod
#     def ring(num: int, radius: float, altitude: float) -> List[Tuple[float, float, float]]:
#         return [(
#             math.cos(2*math.pi*i/num)*radius,
#             math.sin(2*math.pi*i/num)*radius,
#             altitude
#         ) for i in range(num)]

#     @staticmethod
#     def ellipse(num: int, a: float, b: float, altitude: float) -> List[Tuple[float, float, float]]:
#         return [(
#             a*math.cos(2*math.pi*i/num),
#             b*math.sin(2*math.pi*i/num),
#             altitude
#         ) for i in range(num)]

#     @staticmethod
#     def hex_grid(rows: int, cols: int, spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
#         positions = []
#         v_offset = math.sqrt(3)/2 * spacing
#         # center grid
#         width = (cols-1) * spacing
#         height = (rows-1) * v_offset
#         x0 = -width/2
#         y0 = -height/2
#         for i in range(rows):
#             h_shift = (spacing/2) if (i%2) else 0.0
#             for j in range(cols):
#                 x = x0 + j*spacing + h_shift
#                 y = y0 + i*v_offset
#                 positions.append((x, y, altitude))
#         return positions

#     @staticmethod
#     def rectangle(num: int, width: float, height: float, altitude: float, aspect_ratio_preference: float = 1.0) -> List[Tuple[float, float, float]]:
#         """
#         Generates target positions for a rectangular formation.
#         It tries to distribute drones somewhat evenly along the perimeter.
#         Adjusts distribution based on number of drones and aspect ratio.
#         """
#         positions = []
#         perimeter = 2 * (width + height)

#         if num <= 0:
#             return []
        
#         # Distribute drones along the perimeter
#         # Calculate segments for each side
#         # Prioritize distributing along the longer sides more, but ensure at least 1 per side if possible
#         num_on_width_side = max(1, round(num * (width / perimeter)))
#         num_on_height_side = max(1, round(num * (height / perimeter)))

#         # Ensure total drones match num, distribute remainder
#         total_assigned = 2 * (num_on_width_side + num_on_height_side)
#         remaining = num - total_assigned
        
#         # Simple adjustment to ensure all drones are placed, might not be perfectly even for small N
#         if remaining > 0:
#             # Add to longer sides first
#             if width >= height:
#                 num_on_width_side += math.ceil(remaining / 2)
#                 num_on_height_side += math.floor(remaining / 2)
#             else:
#                 num_on_height_side += math.ceil(remaining / 2)
#                 num_on_width_side += math.floor(remaining / 2)

#         # Ensure minimum 1 drone per corner, so at least 2 per side if N > 4
#         # This simple distribution assumes enough drones for non-empty sides.
#         # For small NUM_DRONES (e.g., 2 or 3), this might result in placing on only one side or corner.
#         # A more robust solution for small N would directly place corners then fill.
        
#         # Top side (x from -width/2 to width/2, y = height/2)
#         if num_on_width_side > 0:
#             for i in range(num_on_width_side):
#                 x = -width/2 + (width / (num_on_width_side + 1)) * (i + 1)
#                 positions.append((x, height/2, altitude))
        
#         # Right side (y from height/2 to -height/2, x = width/2) - exclude top/bottom corners if already placed
#         if num_on_height_side > 0:
#             for i in range(num_on_height_side):
#                 y = height/2 - (height / (num_on_height_side + 1)) * (i + 1)
#                 positions.append((width/2, y, altitude))
        
#         # Bottom side (x from width/2 to -width/2, y = -height/2) - exclude corners
#         if num_on_width_side > 0:
#             for i in range(num_on_width_side):
#                 x = width/2 - (width / (num_on_width_side + 1)) * (i + 1)
#                 positions.append((x, -height/2, altitude))

#         # Left side (y from -height/2 to height/2, x = -width/2) - exclude corners
#         if num_on_height_side > 0:
#             for i in range(num_on_height_side):
#                 y = -height/2 + (height / (num_on_height_side + 1)) * (i + 1)
#                 positions.append((-width/2, y, altitude))
        
#         # If num is smaller than total assigned, truncate. If larger, this simple
#         # distribution might under-fill if `num_on_side` is not precisely calculated to sum to `num`.
#         # For simplicity, we'll take the first `num` positions generated.
#         return positions[:num]



# approach 2



# sim_core/formation_planner.py
"""
Defines various formation target generators: wall, ring, ellipse, hex-grid, rectangle, and staggered pattern.
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
        
        # Calculate drones per side
        # Prioritize distributing along the longer sides more, but ensure at least 1 per side if possible
        num_on_width_side = max(1, round(num * (width / perimeter)))
        num_on_height_side = max(1, round(num * (height / perimeter)))

        # Ensure total drones match num, distribute remainder
        total_assigned = 2 * (num_on_width_side + num_on_height_side)
        remaining = num - total_assigned
        
        if remaining > 0:
            if width >= height:
                num_on_width_side += math.ceil(remaining / 2)
                num_on_height_side += math.floor(remaining / 2)
            else:
                num_on_height_side += math.ceil(remaining / 2)
                num_on_width_side += math.floor(remaining / 2)

        # Ensure minimum 1 drone per corner, so at least 2 per side if N > 4
        
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
        
        return positions[:num] # Return exactly 'num' drones

    @staticmethod
    def staggered_pattern(num_drones: int, horizontal_spacing: float, vertical_spacing: float, altitude: float) -> List[Tuple[float, float, float]]:
        """
        Generates target positions for a 2-row staggered pattern based on the provided image.
        Row 1 has ceil(num_drones/2) drones.
        Row 2 has floor(num_drones/2) drones, offset horizontally to fill gaps.
        The entire pattern is centered around (0,0,altitude).
        """
        targets = []
        
        if num_drones <= 0:
            return []
        
        drones_in_row1 = math.ceil(num_drones / 2)
        drones_in_row2 = math.floor(num_drones / 2)

        # Calculate the Y-coordinates for the two rows, centered vertically around 0
        # Row 1 will be slightly behind the Y-axis (negative Y)
        # Row 2 will be slightly in front of the Y-axis (positive Y)
        y_row1 = -vertical_spacing / 2
        y_row2 = vertical_spacing / 2

        # --- Generate Row 1 (main row) ---
        # Calculate the total width of Row 1 for centering
        row1_total_width = (drones_in_row1 - 1) * horizontal_spacing if drones_in_row1 > 1 else 0
        x_start_row1 = -row1_total_width / 2

        for j in range(drones_in_row1):
            x = x_start_row1 + j * horizontal_spacing
            targets.append((x, y_row1, altitude))

        # --- Generate Row 2 (offset row to fill gaps) ---
        # Row 2 will have one fewer drone than Row 1 if num_drones is odd, or equal if even.
        # Its first drone will be at x_start_row1 + horizontal_spacing / 2
        
        # Calculate the starting X for Row 2 relative to the overall pattern center.
        # It's horizontally offset by half the spacing from the start of Row 1.
        x_start_row2 = x_start_row1 + horizontal_spacing / 2

        for j in range(drones_in_row2):
            x = x_start_row2 + j * horizontal_spacing
            targets.append((x, y_row2, altitude))
        
        # Ensure exactly num_drones are returned in case of floating point quirks or small N
        return targets[:num_drones]


