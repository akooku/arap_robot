import cv2
import numpy as np
import sys
import yaml

# File paths (edit these if your filenames are different)
original_path = '/home/aix/ros2_ws/src/arap_robot/arap_robot_navigation/maps/my_cafe_map.pgm'
cropped_path = '/home/aix/ros2_ws/src/arap_robot/arap_robot_navigation/maps/my_cafe_map_edited.pgm'
yaml_path = '/home/aix/ros2_ws/src/arap_robot/arap_robot_navigation/maps/my_cafe_map.yaml'  # The original YAML

# Load images
orig = cv2.imread(original_path, cv2.IMREAD_UNCHANGED)
crop = cv2.imread(cropped_path, cv2.IMREAD_UNCHANGED)

if orig is None or crop is None:
    print("Error: Could not load one or both images.")
    sys.exit(1)

# Find where the cropped image matches the original
result = cv2.matchTemplate(orig, crop, cv2.TM_SQDIFF)
min_val, max_val, min_loc, max_loc = cv2.minMaxLoc(result)
x0, y0 = min_loc  # Top-left corner in original image

print(f"Cropped image offset: x0={x0}, y0={y0}")

# Read the original YAML to get resolution and origin
with open(yaml_path, 'r') as f:
    data = yaml.safe_load(f)
resolution = float(data['resolution'])
old_origin = data['origin']

# Calculate new origin
new_origin_x = old_origin[0] + x0 * resolution
new_origin_y = old_origin[1] + y0 * resolution
theta = old_origin[2] if len(old_origin) > 2 else 0.0

print("\n--- New YAML entry ---")
print(f"image: {cropped_path}")
print(f"resolution: {resolution}")
print(f"origin: [{new_origin_x}, {new_origin_y}, {theta}]")
print(f"negate: {data['negate']}")
print(f"occupied_thresh: {data['occupied_thresh']}")
print(f"free_thresh: {data['free_thresh']}")