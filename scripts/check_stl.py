import struct
import sys

def get_stl_bounds(filepath):
    min_vals = [float('inf'), float('inf'), float('inf')]
    max_vals = [float('-inf'), float('-inf'), float('-inf')]
    
    with open(filepath, 'rb') as f:
        header = f.read(80)
        num_triangles = struct.unpack('<I', f.read(4))[0]
        
        for _ in range(num_triangles):
            # Normal
            f.read(12)
            # 3 vertices
            for _ in range(3):
                v = struct.unpack('<fff', f.read(12))
                for i in range(3):
                    if v[i] < min_vals[i]: min_vals[i] = v[i]
                    if v[i] > max_vals[i]: max_vals[i] = v[i]
            # Attribute byte count
            f.read(2)
            
    return min_vals, max_vals

filepath = r"d:\Programming\RAMI_Project\indoor_space_urdf_description\meshes\toggle_switch_H20_corridor-lamp_3_1.stl"
min_v, max_v = get_stl_bounds(filepath)
print(f"STL Bounding Box:")
print(f"Min: {min_v}")
print(f"Max: {max_v}")
center = [(min_v[i]+max_v[i])/2 for i in range(3)]
print(f"Center: {center}")
