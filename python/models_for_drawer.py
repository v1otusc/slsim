import numpy as np
import math
# import random
# import matplotlib.pyplot as plt

def create_model(ax, pos, direction, up, size=0.01):
    """
    create laser or camera model
    """
    pos = np.array(pos, dtype=float)
    direction = np.array(direction, dtype=float)
    up = np.array(up, dtype=float)

    right = np.cross(direction, up)
    right = right / np.linalg.norm(right)

    up = np.cross(right, direction)
    up = up / np.linalg.norm(up)

    corners = [
        pos + size * (-0.005 * right - 0.005 * up),
        pos + size * (0.005 * right - 0.005 * up),
        pos + size * (0.005 * right + 0.005 * up),
        pos + size * (-0.005 * right + 0.005 * up)
    ]

    for i in range(4):
        ax.plot([corners[i][0], corners[(i+1)%4][0]],
                [corners[i][1], corners[(i+1)%4][1]],
                [corners[i][2], corners[(i+1)%4][2]], 'k-')
        ax.plot(
            [pos[0], corners[i][0]],
            [pos[1], corners[i][1]],
            [pos[2], corners[i][2]],
            "k-",
        )

    direction = direction / np.linalg.norm(direction)
    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        direction[0],
        direction[1],
        direction[2],
        color="r",
        length=0.005,
        normalize=True,
    )


def create_simple_line_laser(ax, pos, direction, up, size=0.01):
    """
    create simple line laser model without considering fov
    """
    pos = np.array(pos, dtype = float)
    direction = np.array(direction, dtype = float)
    up = np.array(up, dtype=float)
    
    right = np.cross(direction, up)
    right = right / np.linalg.norm(right)

    up = np.cross(right, direction)
    up = up / np.linalg.norm(up)
    
    corners = [
        pos + size * (-0.2 * right - 0.2 * up),
        pos + size * (0.2 * right - 0.2 * up),
        pos + size * (0.2 * right + 0.2 * up),
        pos + size * (-0.2 * right + 0.2 * up)
    ]
    
    for i in range(4):
        ax.plot([corners[i][0], corners[(i+1)%4][0]],
                [corners[i][1], corners[(i+1)%4][1]],
                [corners[i][2], corners[(i+1)%4][2]], 'k-')
        ax.plot([pos[0], corners[i][0]], [pos[1], corners[i][1]], [pos[2], corners[i][2]], 'k-')
    
    ax.quiver(pos[0], pos[1], pos[2], direction[0], direction[1], direction[2], color='r', length=size * 2, normalize=True)


def create_line_laser_with_fov(ax, pos, direction, up, size=0.01, fov=120):
    pos = np.array(pos, dtype=float)
    direction = np.array(direction, dtype=float)
    up = np.array(up, dtype=float)

    right = np.cross(direction, up)
    right = right / np.linalg.norm(right)

    up = np.cross(right, direction)
    up = up / np.linalg.norm(up)

    corners = [
        pos + size * (-0.2 * right - 0.2 * up),
        pos + size * (0.2 * right - 0.2 * up),
        pos + size * (0.2 * right + 0.2 * up),
        pos + size * (-0.2 * right + 0.2 * up)
    ]

    for i in range(4):
        ax.plot([corners[i][0], corners[(i+1)%4][0]],
                [corners[i][1], corners[(i+1)%4][1]],
                [corners[i][2], corners[(i+1)%4][2]], 'k-')
        ax.plot([pos[0], corners[i][0]], [pos[1], corners[i][1]], [pos[2], corners[i][2]], 'k-')

    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        direction[0],
        direction[1],
        direction[2],
        color="r",
        length=size * 2,
        normalize=True,
    )

    fov_rad = np.radians(fov / 2)
    fov_distance = size * 3 

    fov_center = pos + fov_distance * direction
    fov_width = 2 * fov_distance * np.tan(fov_rad)
    fov_corners = [
        fov_center + (-0.5 * fov_width * right - 0.5 * fov_width * up),
        fov_center + (0.5 * fov_width * right - 0.5 * fov_width * up),
        fov_center + (0.5 * fov_width * right + 0.5 * fov_width * up),
        fov_center + (-0.5 * fov_width * right + 0.5 * fov_width * up)
    ]

    for i in range(4):
        ax.plot([fov_corners[i][0], fov_corners[(i+1)%4][0]],
                [fov_corners[i][1], fov_corners[(i+1)%4][1]],
                [fov_corners[i][2], fov_corners[(i+1)%4][2]], 'g--')


def create_camera_cuboid(ax, pos, direction, up, width=0.2, height=0.15, depth=0.1):
    pos = np.array(pos, dtype=float)
    direction = np.array(direction, dtype=float)
    up = np.array(up, dtype=float)

    right = np.cross(direction, up)
    right = right / np.linalg.norm(right)

    up = np.cross(right, direction)
    up = up / np.linalg.norm(up)

    corners = [
        pos + 0.5 * (-width * right - height * up - depth * direction),
        pos + 0.5 * (width * right - height * up - depth * direction),
        pos + 0.5 * (width * right + height * up - depth * direction),
        pos + 0.5 * (-width * right + height * up - depth * direction),
        pos + 0.5 * (-width * right - height * up + depth * direction),
        pos + 0.5 * (width * right - height * up + depth * direction),
        pos + 0.5 * (width * right + height * up + depth * direction),
        pos + 0.5 * (-width * right + height * up + depth * direction)
    ]

    edges = [
        (0,1), (1,2), (2,3), (3,0), 
        (4,5), (5,6), (6,7), (7,4),
        (0,4), (1,5), (2,6), (3,7) 
    ]

    for edge in edges:
        ax.plot([corners[edge[0]][0], corners[edge[1]][0]],
                [corners[edge[0]][1], corners[edge[1]][1]],
                [corners[edge[0]][2], corners[edge[1]][2]], 'k-')

    direction_length = max(width, height, depth)
    ax.quiver(pos[0], pos[1], pos[2], 
              direction[0], direction[1], direction[2], 
              color='r', length=direction_length, normalize=True)


def create_camera_with_fov(ax, pos, direction, up, size=0.005, fov_h=100, fov_v=78):
    """
    create camera model with rectangular FOV
    """
    pos = np.array(pos, dtype=float)
    direction = np.array(direction, dtype=float)
    up = np.array(up, dtype=float)

    right = np.cross(direction, up)
    right = right / np.linalg.norm(right)

    up = np.cross(right, direction)
    up = up / np.linalg.norm(up)

    corners = [
        pos + size * (-0.2 * right - 0.2 * up),
        pos + size * (0.2 * right - 0.2 * up),
        pos + size * (0.2 * right + 0.2 * up),
        pos + size * (-0.2 * right + 0.2 * up),
    ]

    for i in range(4):
        ax.plot([corners[i][0], corners[(i+1)%4][0]],
                [corners[i][1], corners[(i+1)%4][1]],
                [corners[i][2], corners[(i+1)%4][2]], 'k-')
        ax.plot([pos[0], corners[i][0]], [pos[1], corners[i][1]], [pos[2], corners[i][2]], 'k-')

    ax.quiver(
        pos[0],
        pos[1],
        pos[2],
        direction[0],
        direction[1],
        direction[2],
        color="g",
        length=size * 2,
        normalize=True,
    )

    fov_h_rad = np.radians(fov_h / 2)
    fov_v_rad = np.radians(fov_v / 2)
    fov_distance = size * 8 

    fov_center = pos + fov_distance * direction
    fov_width = 2 * fov_distance * np.tan(fov_h_rad)
    fov_height = 2 * fov_distance * np.tan(fov_v_rad)
    fov_corners = [
        fov_center + (-0.5 * fov_width * right - 0.5 * fov_height * up),
        fov_center + (0.5 * fov_width * right - 0.5 * fov_height * up),
        fov_center + (0.5 * fov_width * right + 0.5 * fov_height * up),
        fov_center + (-0.5 * fov_width * right + 0.5 * fov_height * up)
    ]

    for i in range(4):
        ax.plot([fov_corners[i][0], fov_corners[(i+1)%4][0]],
                [fov_corners[i][1], fov_corners[(i+1)%4][1]],
                [fov_corners[i][2], fov_corners[(i+1)%4][2]], 'g--')

    for corner in fov_corners:
        ax.plot([pos[0], corner[0]], [pos[1], corner[1]], [pos[2], corner[2]], "g--")

# computational geometry functions

eps = 1e-6

class Node:
    def __init__(self, x, y, z):
        self.x = x
        self.y = y
        self.z = z

    def __len__(self):
        return (self.x**2 + self.y**2 + self.z**2)**0.5

    def __add__(self, other):
        if isinstance(other, Node):
            return Node(self.x + other.x, self.y + other.y, self.z + other.z)
        else:
            raise TypeError(
                "unsupported operand type(s) for +: 'Node' and '{}'".format(
                    type(other).__name__
                )
            )

    def __radd__(self, other):
        if other == 0:
            return self
        else:
            return self.__add__(other)

    def __sub__(self, other):
        return Node(self.x - other.x, self.y - other.y, self.z - other.z)

    def __mul__(self, scalar):
        return Node(self.x * scalar, self.y * scalar, self.z * scalar)

    def __rmul__(self, scalar):
        return self.__mul__(scalar)

    def __truediv__(self, scalar):
        return Node(self.x / scalar, self.y / scalar, self.z / scalar)
    
    def __repr__(self): # -> str:
        return "Node({}, {}, {})".format(self.x, self.y, self.z)

    @classmethod
    def mean(cls, nodes):
        sum_node = sum(nodes, cls(0, 0, 0))
        return sum_node / len(nodes)

    def cross(self, other):
        return Node(
            self.y * other.z - self.z * other.y,
            self.z * other.x - self.x * other.z,
            self.x * other.y - self.y * other.x,
        )

    def dot(self, other):
        return self.x * other.x + self.y * other.y + self.z * other.z

    def normalize(self):
        length = (self.x**2 + self.y**2 + self.z**2)**0.5
        if length < eps:
            raise ValueError("Cannot normalize zero vector")
        inv_length = 1.0 / length 
        return Node(self.x * inv_length, self.y * inv_length, self.z * inv_length)

    def to_array(self):
        return np.array([self.x, self.y, self.z])

class Face:
    def __init__(self, nodes):
        if len(nodes) < 3:
            raise ValueError("Face must have at least 3 nodes")
        self.nodes = nodes

    def normal(self, debug=False):
        v1 = self.nodes[1] - self.nodes[0]
        v2 = self.nodes[2] - self.nodes[0]
        if(debug):
            print("v1 -- ", v1)
            print("v2 -- ", v2)
        return v1.cross(v2)

    def area(self):
        if len(self.nodes) == 3:
            normal = self.normal()
            return 0.5 * (len(normal))
        else:
            raise NotImplementedError(
                "Area calculation for non-triangular faces is not implemented"
            )

    def point(self):
        return self.nodes[0]


def convex_hull_2d(xs, ys):
    """
    TODO: add 2d convex hull 
    """

def convex_hull_3d(xs, ys, zs):
    """
    3d convex hull code from
    https://oi-wiki.org/geometry/convex-hull/
    """
    def _is_coplanar(p1, p2, p3, p4):
        v1 = p2 - p1
        v2 = p3 - p1
        v3 = p4 - p1
        return abs(v1.dot(v2.cross(v3))) < eps

    def _find_initial_tetrahedron(Nodes):
        for i in range(len(Nodes)):
            for j in range(i+1, len(Nodes)):
                for k in range(j+1, len(Nodes)):
                    for l in range(k+1, len(Nodes)):
                        if not _is_coplanar(Nodes[i], Nodes[j], Nodes[k], Nodes[l]):
                            return [Nodes[i], Nodes[j], Nodes[k], Nodes[l]] 

    def _orient_tetrahedron(Nodes):
        def _volume(a, b, c, d):
            return (b - a).cross(c - a).dot(d - a)

        faces = [
            (Nodes[0], Nodes[1], Nodes[2]),
            (Nodes[0], Nodes[2], Nodes[3]),
            (Nodes[0], Nodes[3], Nodes[1]),
            (Nodes[1], Nodes[3], Nodes[2])
        ]

        opposite_nodes = [Nodes[3], Nodes[1], Nodes[2], Nodes[0]]
        oriented_faces = []
        for face, opposite_node in zip(faces, opposite_nodes):
            a, b, c = face
            # make sure the normal of each face points outward
            if _volume(a, b, c, opposite_node) > 0:
                oriented_faces.append((a, c, b))
            else:
                oriented_faces.append(face)

        return [Face(face) for face in oriented_faces]

    def _can_see(face: Face, node: Node) -> bool:
        """
        Normal of the face points outward
        """
        return face.normal().dot(node - face.nodes[0]) > eps

    def _calculate_3d_center(faces):
        all_nodes = set(node for face in faces for node in face.nodes)
        return Node(
            sum(n.x for n in all_nodes) / len(all_nodes),
            sum(n.y for n in all_nodes) / len(all_nodes),
            sum(n.z for n in all_nodes) / len(all_nodes)
        )

    def _if_face_outward(face, center):
        normal = face.normal()
        return normal.dot(center - face.nodes[0]) < 0

    nodes = [Node(x, y, z) for x, y, z in zip(xs, ys, zs)] 

    if len(nodes) < 4:
        raise ValueError("Nodes must have at least 4 points for 3D convex hull")

    initial_nodes = _find_initial_tetrahedron(nodes)
    faces = _orient_tetrahedron(initial_nodes)

    # incremental finding algorithm
    for node in nodes:
        if node in initial_nodes:
            continue

        visible_faces = [face for face in faces if _can_see(face, node)]
        if not visible_faces:
            continue

        edges = set()
        for face in visible_faces:
            faces.remove(face)
            edges.update(
                tuple(sorted((face.nodes[i], face.nodes[(i + 1) % 3]), key=id))
                for i in range(3)
            )

        center = _calculate_3d_center(faces)
        for edge in edges:
            new_face = Face([edge[0], edge[1], node])
            if _if_face_outward(new_face, center):
                new_face = Face([edge[1], edge[0], node])
            faces.append(new_face)

    return faces


def edges_of_faces(faces):
    edges = set()
    for face in faces:
        for j in range(3):
            edge = tuple(sorted((face.nodes[j], face.nodes[(j+1)%3]), key=id))
            edges.add(edge)
    return edges


def _intersect_point_of_line_plane(edge, face, debug=False):
    """
    \brief -- Find the intersection point of a line and a plane

    Plane Equation:
    Suppose the equation of the plane is ax + by + cz + d = 0, where (a, b, c) is the normal vector to the plane.

    Line equation:
    P(t) = P1 + t(P2 - P1), where P1 is the starting point, P2 is the ending point, and 0 ≤ t ≤ 1.

    Intersection calculation:
    Substitute the equation of the line into the equation of the plane:
    a(x1 + t(x2-x1)) + b(y1 + t(y2-y1)) + c(z1 + t(z2-z1)) + d = 0

    Expand and rearrange:
    (ax1 + by1 + cz1 + d) + t(a(x2-x1) + b(y2-y1) + c(z2-z1)) + d = 0            --  (1)

    plane.point() on the plane, we can get:
    ax0 + by0 + cz0 + d = 0                                                      --  (2)
    
    so we can get:
    a(x1 - x0) + b(y1 - y0) + c(z1 - z0) + t(a(x2-x1) + b(y2-y1) + c(z2-z1)) = 0 --  (3)

    toggle negative sign 
    t = a(x1 - x0) + b(y1 - y0) + c(z1 - z0) / a(x1-x2) + b(y1-y2) + c(z1-z2)    --  (4)

    Corresponding to the code:

    u = edge[0] - edge[1] corresponds to (x1-x2, y1-y2, z1-z2)
    w = edge[0] - plane.point() corresponds to (x1-x0, y1-y0, z1-z0) where (x0, y0, z0) is a point on the plane
    D = plane.normal().dot(u) corresponds to a(x1-x2) + b(y1-y2) + c(z1-z2)
    N = plane.normal().dot(w) corresponds to a(x1-x0) + b(y1-y0) + c(z1-z0)
    
    so we can get:
    t = N / D                                                                    -- *(4)*

    This corresponds to sI = N / D in the code
    """
    if(debug):
        print("\n")
        print("-"*50)
        print("compute intersection point of line and plane")
        print("edge[0] -- ", edge[0])
        print("edge[1] -- ", edge[1])

    # u = (x1-x2, y1-y2, z1-z2)
    u = edge[0] - edge[1]

    if debug:
        print("u -- ", u)

    # w = (x1-x0, y1-y0, z1-z0)
    w = edge[0] - face.point()

    if(debug):
        print("w -- ", w)
        print("face normal -- ", face.normal())

    # D = a(x1-x2) + b(y1-y2) + c(z1-z2)
    D = face.normal().dot(u)
    
    if(debug):
        print("D -- ", D)

    # N = (a(x1-x0) + b(y1-y0) + c(z1-z0))
    N = face.normal().dot(w)

    if(debug):
        print("N -- ", N)

    if abs(D) < eps:
        if N == 0:
            # on the plane
            return True, edge[0]
        else:
            return False, None
        
    # sI = (a(x1-x0) + b(y1-y0) + c(z1-z0)) / (a(x1-x2) + b(y1-y2) + c(z1-z2)
    sI = N / D

    if(debug):
        print("sI -- ", sI)

    if sI < -eps or sI > 1 + eps:
        return False, None

    # (x1 + t(x2-x1), y1 + t(y2-y1), z1 + t(z2-z1)))
    return True, edge[0] - sI * u


def compute_convex_hull_plane_intersection(convex_hull, plane):
    intersection_points = []

    for edge in convex_hull:
        intersects, point = _intersect_point_of_line_plane(edge, plane)
        if intersects:
            intersection_points.append(point)

    print("len of intersection_points: ", len(intersection_points))

    # remove duplicate points
    # No near-equal points were found before adding to unique_points
    unique_points = []
    for point in intersection_points:
        if not any(np.allclose(point.to_array(), p.to_array()) for p in unique_points):
            unique_points.append(point)

    # sort points to form a closed polygon
    if len(unique_points) > 2:
        center = Node.mean(unique_points)
        v1 = unique_points[0] - center
        v2 = plane.normal().cross(v1)

        def sort_key(point):
            v = point - center
            angle = math.atan2(v.dot(v2), v.dot(v1))
            return angle

        sorted_points = sorted(unique_points, key=sort_key)
        print("len of sorted_points(unique_points): ", len(sorted_points))
        return sorted_points

    print("len of unique_points: ", len(unique_points))
    return unique_points
