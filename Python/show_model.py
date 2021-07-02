import numpy as np
import open3d as o3d

# Carlos Murillo
# 400197550
# murillc
# Python 3.7

# Read point cloud
# File is formatted as such:
# [co-ordinate][slice index][point index]
# x0-0 y0-0 z0-0
# ...
# x0-63 y0-63 z0-63
# x1-0 y1-0 z1-0
# ...
print("Testing IO for point cloud ...")
pcd = o3d.io.read_point_cloud("room.xyz", format='xyz')
print(pcd)

# o3d.visualization.draw_geometries([pcd])

# Print array of points to console
print(np.asarray(pcd.points))

points_array = np.asarray(pcd.points) # array of points
lines = [] # lineset

slices = 12 # number of y-z slices
points = 64 # number of points in each y-z slice

# Loop through y-z slices
for j in range(0, points*slices, points):
    # Loop through each point in each slice
    for i in range(points-1):
        lines.append([i+j, i+j+1])
        print(str(i+j) + ", " + str(i+j+1))

        # If not on the last slice, connect point to its equivalent on next slice
        if j != points*(slices-1):
            lines.append([i+j, i+j+points])
    lines.append([(points-1)+j, j])

line_set = o3d.geometry.LineSet(points=o3d.utility.Vector3dVector(points_array), lines=o3d.utility.Vector2iVector(lines))

# Co-ordinate axes
mesh_frame = o3d.geometry.TriangleMesh.create_coordinate_frame(size=500, origin=[0, 0, 0])

# Without coord axes
o3d.visualization.draw_geometries([line_set])

# With coord axes
# o3d.visualization.draw_geometries([line_set, mesh_frame])
