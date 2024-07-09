import numpy as np
from typing import List, Tuple

class Plane:
    def __init__(self, point: np.ndarray, normal: np.ndarray):
        self.point = point
        self.normal = normal / np.linalg.norm(normal)  # 归一化法向量

class Edge:
    def __init__(self, start: np.ndarray, end: np.ndarray):
        self.start = start
        self.end = end

def intersect_line_plane(edge: Edge, plane: Plane) -> Tuple[bool, np.ndarray]:
    u = edge.end - edge.start
    w = edge.start - plane.point
    
    D = np.dot(plane.normal, u)
    N = -np.dot(plane.normal, w)
    
    if abs(D) < 1e-7:  # 线段平行于平面
        return False, None
    
    sI = N / D
    if sI < 0 or sI > 1:  # 交点不在线段上
        return False, None
    
    return True, edge.start + sI * u

def compute_convex_hull_plane_intersection(convex_hull: List[Edge], plane: Plane) -> List[np.ndarray]:
    intersection_points = []
    
    for edge in convex_hull:
        intersects, point = intersect_line_plane(edge, plane)
        if intersects:
            intersection_points.append(point)
    
    # 去除重复点
    unique_points = []
    for point in intersection_points:
        if not any(np.allclose(point, p) for p in unique_points):
            unique_points.append(point)
    
    # 对点进行排序以形成一个闭合的多边形
    if len(unique_points) > 2:
        center = np.mean(unique_points, axis=0)
        v1 = unique_points[0] - center
        v2 = np.cross(plane.normal, v1)
        
        def sort_key(point):
            v = point - center
            angle = np.arctan2(np.dot(v, v2), np.dot(v, v1))
            return angle
        
        sorted_points = sorted(unique_points, key=sort_key)
        return sorted_points
    
    return unique_points

# 示例使用
# 定义凸包（这里用一个立方体作为例子）
cube_vertices = [
    np.array([0, 0, 0]), np.array([1, 0, 0]), np.array([1, 1, 0]), np.array([0, 1, 0]),
    np.array([0, 0, 1]), np.array([1, 0, 1]), np.array([1, 1, 1]), np.array([0, 1, 1])
]

cube_edges = [
    Edge(cube_vertices[0], cube_vertices[1]), Edge(cube_vertices[1], cube_vertices[2]),
    Edge(cube_vertices[2], cube_vertices[3]), Edge(cube_vertices[3], cube_vertices[0]),
    Edge(cube_vertices[4], cube_vertices[5]), Edge(cube_vertices[5], cube_vertices[6]),
    Edge(cube_vertices[6], cube_vertices[7]), Edge(cube_vertices[7], cube_vertices[4]),
    Edge(cube_vertices[0], cube_vertices[4]), Edge(cube_vertices[1], cube_vertices[5]),
    Edge(cube_vertices[2], cube_vertices[6]), Edge(cube_vertices[3], cube_vertices[7])
]

# 定义平面
plane = Plane(np.array([0.5, 0.5, 0.5]), np.array([1, 1, 1]))

# 计算交线
intersection = compute_convex_hull_plane_intersection(cube_edges, plane)

print("交点:")
for point in intersection:
    print(point)