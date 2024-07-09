#!/usr/bin/env python

# import os
import math
import numpy as np
import matplotlib as mpl
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d import proj3d

from models_for_drawer import (
    create_camera_with_fov,
    create_simple_line_laser,
    Node,
    Face,
    convex_hull_3d,
    edges_of_faces,
    compute_convex_hull_plane_intersection,
)

# create_line_laser_with_fov

ground_pt_x = []
ground_pt_y = []
ground_pt_z = []

cam_pt_x    = []
cam_pt_y    = []
cam_pt_z    = []

image_ground_pt_x  = []
image_ground_pt_y  = []

w_obstacles_pt_x = []
w_obstacles_pt_y = []
w_obstacles_pt_z = []

params = {}

with open('../data/gend/meta.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        if line.startswith('#'):
            line = line[2:].lstrip().rstrip()
            if len(line.split()) == 2:
                key, value = line.split()
                params[key] = float(value)


with open('../data/gend/ground_pts_in_world.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        if line.startswith('#'):
            continue
        pt = [float(x) for x in line.split()]
        ground_pt_x.append(pt[0])
        ground_pt_y.append(pt[1])
        ground_pt_z.append(pt[2])


with open('../data/gend/ground_pts_in_cam.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        if line.startswith('#'):
            continue
        pt = [float(x) for x in pt]
        ground_pt_x.append(pt[0])
        ground_pt_y.append(pt[1])
        ground_pt_z.append(pt[2])


with open('../data/gend/ground_pts_in_image.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        if line.startswith('#'):
            continue
        pt = [float(x) for x in line.split()]
        image_ground_pt_x.append(pt[0])
        image_ground_pt_y.append(pt[1])


with open('../data/obstacle.txt', 'r') as f:
    data = f.readlines()
    for line in data:
        if line.startswith('#'):
            continue
        pt = [float(x) for x in line.split()]
        w_obstacles_pt_x.append(pt[0])
        w_obstacles_pt_y.append(pt[1])
        w_obstacles_pt_z.append(pt[2])


# -----------------------------------------
# plot 3d
fig = plt.figure(figsize=(24, 8))
plt.ion()

if int(mpl.__version__[0]) > 2:
    ax = fig.add_subplot(1, 2, 1, projection='3d')
    ax.set_position([0.05, 0.1, 0.4, 0.8])  # [left, bottom, width, height] of ax
else:
    ax = fig.gca(projection='3d')

xlim_min = 0
xlim_max = 0.6
ylim_min = -0.6
ylim_max = 0.6
zlim_min = -0.2
zlim_max = 0.4

xlim_min = min(xlim_min, min(w_obstacles_pt_x) - 0.1)
xlim_max = max(xlim_max, max(w_obstacles_pt_x) + 0.1)
ylim_min = min(ylim_min, min(w_obstacles_pt_y) - 0.1)
ylim_max = max(ylim_max, max(w_obstacles_pt_y) + 0.1)
zlim_min = min(zlim_min, min(w_obstacles_pt_z) - 0.1)
zlim_max = max(zlim_max, max(w_obstacles_pt_z) + 0.1)

ax.set_xlabel('X -- m')
ax.set_ylabel('Y -- m')
ax.set_zlabel('Z -- m')
ax.set_xlim(xlim_min, xlim_max)
ax.set_ylim(ylim_min, ylim_max)
ax.set_zlim(zlim_min, zlim_max)
ax.set_aspect('equal', 'box')

x = np.arange(xlim_min, xlim_max, 0.01)
y = np.arange(ylim_min, ylim_max, 0.01)
X, Y = np.meshgrid(x, y)
Z = np.zeros_like(X)
ax.plot_surface(X, Y, Z, alpha=0.1, color='gray')

# plot camera fov model
camera_pos = np.array([0, 0, params['hc']])
camera_direction = np.array([1, 0, 0])
camera_up = np.array([0, 0, 1])
create_camera_with_fov(
    ax,
    camera_pos,
    camera_direction,
    camera_up,
    size=0.05,
    fov_h=params["hf"],
    fov_v=params["vf"],
)

# plot laser model
laser_pos = np.array([0, 0, float(params['hl'])])
laser_direction = np.array(
    [ math.cos(float(params["alpha"]) * math.pi / 180), 0, -math.sin(float(params["alpha"]) * math.pi / 180) ]
)
laser_up = np.array(
    [ math.sin(float(params["alpha"]) * math.pi / 180), 0,  math.cos(float(params["alpha"]) * math.pi / 180) ]
)

# calc two end points of laser
min_ground_y = min(ground_pt_y)
max_ground_y = max(ground_pt_y)
laser_end_point_index1 = ground_pt_y.index(min_ground_y)
laser_end_point_index2 = ground_pt_y.index(max_ground_y)
create_simple_line_laser(ax, laser_pos, laser_direction, laser_up, size=0.05)
ax.plot(
    [laser_pos[0], ground_pt_x[laser_end_point_index1]],
    [laser_pos[1], ground_pt_y[laser_end_point_index1]],
    [laser_pos[2], ground_pt_z[laser_end_point_index1]],
    "b--"
)
ax.plot(
    [laser_pos[0], ground_pt_x[laser_end_point_index2]],
    [laser_pos[1], ground_pt_y[laser_end_point_index2]],
    [laser_pos[2], ground_pt_z[laser_end_point_index2]],
    "b--"
)

# plot laser points on the ground
ax.scatter(ground_pt_x, ground_pt_y, ground_pt_z, c="b", s=0.1, label="ground points")

# plot raw obstacles pts in world frame
ax.scatter(
    w_obstacles_pt_x,
    w_obstacles_pt_y,
    w_obstacles_pt_z,
    c="r",
    s=6,
    marker="x",
    label="obstacles points",
)

# plot convex hull of raw obstacles pts
convex_hull_faces = convex_hull_3d(w_obstacles_pt_x, w_obstacles_pt_y, w_obstacles_pt_z)
convex_hull_edges = edges_of_faces(convex_hull_faces)

print("%d faces of convex hulls found from input obstacles pts in world frame: " % len(convex_hull_faces))
print("%d edges of convex hulls found from input obstacles pts in world frame: " % len(convex_hull_edges))

for edge in convex_hull_edges:
    # print(edge[0].x, edge[0].y, edge[0].z, edge[1].x, edge[1].y, edge[1].z)
    ax.plot(
        [edge[0].x, edge[1].x],
        [edge[0].y, edge[1].y],
        [edge[0].z, edge[1].z],
        "r",
        linewidth=0.5,
    )

laser_plane = Face(
    [
        Node(laser_pos[0], laser_pos[1], laser_pos[2]),
        Node(
            ground_pt_x[laser_end_point_index1],
            ground_pt_y[laser_end_point_index1],
            ground_pt_z[laser_end_point_index1],
        ),
        Node(
            ground_pt_x[laser_end_point_index2],
            ground_pt_y[laser_end_point_index2],
            ground_pt_z[laser_end_point_index2],
        ),
    ]
)

# ---------------------------------------------
print(
    "laser plane normal -- ",
    laser_plane.normal().x,
    laser_plane.normal().y,
    laser_plane.normal().z,
)

print(
    "laser plane point -- ",
    laser_plane.point().x,
    laser_plane.point().y,
    laser_plane.point().z,
)
# ---------------------------------------------

can_seen_by_laser_edges = set()
not_seen_by_laser_edges = set()
for face in convex_hull_faces:
    laser_node = Node(laser_pos[0], laser_pos[1], laser_pos[2])
    if face.normal().dot(laser_node - face.nodes[0]) > 1e-6:
        for i in range(3):
            edge = tuple(sorted((face.nodes[i], face.nodes[(i+1)%3]), key=id))
            can_seen_by_laser_edges.add(edge)
    else:
        for i in range(3):
            edge = tuple(sorted((face.nodes[i], face.nodes[(i+1)%3]), key=id))
            not_seen_by_laser_edges.add(edge)

# remove edges that can be seen by laser
# not_seen_by_laser_edges = not_seen_by_laser_edges.difference(can_seen_by_laser_edges)

intersected_points = compute_convex_hull_plane_intersection(
    can_seen_by_laser_edges, laser_plane
)

fake_intersected_points = compute_convex_hull_plane_intersection(
    not_seen_by_laser_edges, laser_plane
)

print(len(intersected_points), "intersected points found")
for i in range(len(intersected_points)):
    ax.scatter(
        intersected_points[i].to_array()[0],
        intersected_points[i].to_array()[1],
        intersected_points[i].to_array()[2],
        c="g",
        s=6,
        marker="x",
        label="intersected points",
    )

for i in range(len(fake_intersected_points)):
    ax.scatter(
        fake_intersected_points[i].to_array()[0],
        fake_intersected_points[i].to_array()[1],
        fake_intersected_points[i].to_array()[2],
        c="g",
        s=6,
        marker="x",
        label="fake intersected points",
    )

for i in range(len(intersected_points)):
    ax.plot(
        [intersected_points[i].x, intersected_points[(i + 1)%(len(intersected_points))].x],
        [intersected_points[i].y, intersected_points[(i + 1)%(len(intersected_points))].y],
        [intersected_points[i].z, intersected_points[(i + 1)%(len(intersected_points))].z],
        "-",
        color="orange",
        linewidth=1,
    )


for i in range(len(fake_intersected_points)):
    ax.plot(
        [
            fake_intersected_points[i].x,
            fake_intersected_points[(i + 1) % (len(fake_intersected_points))].x,
        ],
        [
            fake_intersected_points[i].y,
            fake_intersected_points[(i + 1) % (len(fake_intersected_points))].y,
        ],
        [
            fake_intersected_points[i].z,
            fake_intersected_points[(i + 1) % (len(fake_intersected_points))].z,
        ],
        "--",
        color="orange",
        linewidth=1,
    )

ax.grid(False)

# -----------------------------------------
# gen image points
sorted_intersected_points = sorted(intersected_points, key=lambda x: x.y)

def world_to_camera(x, y, z):
    homogeneous = np.array([x, y, z, 1.0])
    T = np.matrix([[0, -1, 0, 0], [0, 0, -1, params["hc"]], [1, 0, 0, 0], [0, 0, 0, 1]])
    cam_point = T.dot(homogeneous)
    cx, cy, cz = cam_point[0, 0], cam_point[0, 1], cam_point[0, 2]
    return cx, cy, cz

def camera_to_image(cx, cy, cz):
    u = params["fx"] * cx / cz + params["cx"]
    v = params["fy"] * cy / cz + params["cy"]

    if u < 0 or u > 640 or v < 0 or v > 480:
        u = -1
        v = -1

    return u, v


cam_intercepted_pts_x = []
cam_intercepted_pts_y = []
cam_intercepted_pts_z = []
image_intercepted_pts_u = []
image_ingercepted_pts_v = []

for i in range(len(sorted_intersected_points)):
    cam_intercepted_pt_x, cam_intercepted_pt_y, cam_intercepted_pt_z = world_to_camera(
        sorted_intersected_points[i].x,
        sorted_intersected_points[i].y,
        sorted_intersected_points[i].z,
    )

    image_intercepted_pt_u, image_intercepted_pt_v = camera_to_image(
        cam_intercepted_pt_x,
        cam_intercepted_pt_y,
        cam_intercepted_pt_z,
    )

    if image_intercepted_pt_u == -1 or image_intercepted_pt_v == -1:
        continue

    cam_intercepted_pts_x.append(cam_intercepted_pt_x)
    cam_intercepted_pts_y.append(cam_intercepted_pt_y)
    cam_intercepted_pts_z.append(cam_intercepted_pt_z)

    image_intercepted_pts_u.append(image_intercepted_pt_u)
    image_ingercepted_pts_v.append(image_intercepted_pt_v)

# -----------------------------------------
# plot 2d
ax2 = fig.add_subplot(1, 2, 2)

new_image_ground_pt_x = []
new_image_ground_pt_y = []

for x in image_ground_pt_x:
    if x < min(image_intercepted_pts_u) or x > max(image_intercepted_pts_u):
        new_image_ground_pt_x.append(x)
        new_image_ground_pt_y.append(image_ground_pt_y[image_ground_pt_x.index(x)])

image_ground_pt_x = new_image_ground_pt_x
image_ground_pt_y = new_image_ground_pt_y

ax2.scatter(
    image_ground_pt_x, image_ground_pt_y, c="b", marker="o", s=0.5, label="image points"
)

ax2.scatter(
    image_intercepted_pts_u,
    image_ingercepted_pts_v,
    c="g",
    marker="x",
    s=10,
    label="intersected points",
)

ax2.plot(
    image_intercepted_pts_u,
    image_ingercepted_pts_v,
    "orange",
    linewidth=2,
    label="intersected points",
)

ax2.xaxis.set_ticks_position('top')
ax2.xaxis.set_label_position('top')
ax2.yaxis.set_ticks_position('left')
ax2.yaxis.set_label_position('left')
ax2.set_xlabel("U -- pixel")
ax2.set_ylabel("V -- pixel")
ax2.invert_yaxis()
ax2.set_xlim(0, 640)
ax2.set_ylim(480, 0)
ax2.set_aspect('equal', 'box')

# plt.tight_layout()
fig.suptitle('3D - 2D viewer', fontsize=16)
plt.savefig('./simulation.png', dpi=600)
plt.show(block=True)
