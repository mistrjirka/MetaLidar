import math
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # ensures 3D projection support

# Image dimensions and FOV settings
width = 1024
height = 1024
FOVH = math.radians(90)  # horizontal FOV of 90°
FOVV = math.radians(90)  # vertical FOV of 90°
tanFOVH2 = math.tan(FOVH / 2)
tanFOVV2 = math.tan(FOVV / 2)

def compute_pixel(v, h_eff):
    """
    Computes pixel coordinates from vCoord (vertical angle) and h_eff (effective horizontal angle)
    using:
      x = width/2 * (1 + tan(h_eff)/tan(FOVH/2))
      y = height/2 * (1 - cos(v)/(sin(v)*cos(h_eff)*tan(FOVV/2)))
    """
    x = width / 2 * (1 + math.tan(h_eff) / tanFOVH2)
    y = height / 2 * (1 - (math.cos(v) / (math.sin(v) * math.cos(h_eff) * tanFOVV2)))
    return x, y

# Define camera yaw offsets (in radians)
# Front: yaw = 0, Right: yaw = 90°, Back: yaw = 180°, Left: yaw = -90°
camera_orientations = {
    "Front": 0,
    "Right": math.pi/2,
    "Back": math.pi,
    "Left": -math.pi/2,
}

colors_array = {
    "Front": "green",
    "Right": "blue",
    "Back": "orange",
    "Left": "yellow",
}

# Create a grid of global spherical coordinates:
# v (vertical/polar angle): avoid 0 and pi to prevent singularities, use 0.01 to pi-0.01
# h (azimuth): full circle from -pi to pi
v_vals = np.linspace(0.01, math.pi - 0.01, 100)
h_vals = np.linspace(-math.pi, math.pi, 200)
v_grid, h_grid = np.meshgrid(v_vals, h_vals)
v_flat = v_grid.flatten()
h_flat = h_grid.flatten()

# Set up a 2x2 subplot for the four cameras.
fig = plt.figure(figsize=(12, 12))


for i, (cam_name, yaw) in enumerate(camera_orientations.items()):
    ax = fig.add_subplot(2, 2, i + 1, projection='3d')
    
    sphere_x = []
    sphere_y = []
    sphere_z = []
    colors = []
    
    for v, h in zip(v_flat, h_flat):
        # Global spherical coordinates to Cartesian for plotting the sphere.
        X = math.sin(v) * math.cos(h)
        Y = math.sin(v) * math.sin(h)
        Z = math.cos(v)
        
        # Compute effective horizontal angle relative to the camera's orientation.
        h_eff = h - yaw
        
        try:
            x_pixel, y_pixel = compute_pixel(v, h_eff)
            in_bounds = (0 <= x_pixel <= width) and (0 <= y_pixel <= height)
        except Exception as e:
            in_bounds = False
        
        sphere_x.append(X)
        sphere_y.append(Y)
        sphere_z.append(Z)
        colors.append(colors_array[cam_name] if in_bounds else 'red')
    
    scatter = ax.scatter(sphere_x, sphere_y, sphere_z, c=colors, marker='o', s=10)
    ax.set_title(f"{cam_name} Camera (Yaw = {math.degrees(yaw):.0f}°)")
    ax.set_xlabel("X")
    ax.set_ylabel("Y")
    ax.set_zlabel("Z")
    ax.set_box_aspect([1, 1, 1])
    
plt.suptitle("360° Sphere: Green = In Bounds, Red = Out of Bounds", fontsize=16)
plt.show()
