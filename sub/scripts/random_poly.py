from shapely.geometry import Polygon, Point, LineString
from shapely.ops import polygonize
import random
import numpy as np

def check_intersection(set, line):
    set_new = set.copy()[:-1]
    for l in set_new:
        if line.intersects(l):
            return True
    return False

def generate_random_polygon(min_x, max_x, min_y, max_y, num_points, num_holes, seed=0):
    random.seed(seed)
    points = []
    lineset = []
    cur = 0
    for i in range(num_points):
        # print(i, seed)
        if i == 0:
            x = 0
            y = 0
            cur = x
        elif i == num_points - 1:
            x = cur
            y = 0
        elif i % 2 == 1:
            x = cur
            y = random.randint(min_y * 2, max_y * 2) * 0.5
            new_line = LineString((points[i - 1], (x, y)))
            num = 0
            while check_intersection(lineset, new_line) and y != points[-1][1]:
                y = random.randint(min_y * 2, max_y * 2) * 0.5
                new_line = LineString((points[i - 1], (x, y)))
                num += 1
                assert num < 30
            cur = y
            lineset.append(new_line)
        elif i % 2 == 0:
            y = cur
            x = random.randint(min_x * 2, max_x * 2) * 0.5
            new_line = LineString((points[i - 1], (x, y)))
            num = 0
            while check_intersection(lineset, new_line) or x == points[-1][0]:
                x = random.randint(min_x * 2, max_x * 2) * 0.5
                new_line = LineString((points[i - 1], (x, y)))
                num += 1
                assert num < 30
            cur = x
            lineset.append(new_line)
        points.append((x, y))
    polygon = Polygon(points)
    if not polygon.is_valid:
        polygon = polygon.buffer(0)
    fixed_polygons = list(polygonize(polygon.exterior))[0]
    vertices = list(fixed_polygons.exterior.coords)


    # 将顶点坐标转换为2xN的NumPy数组
    vertices_array = np.array(vertices).T

    # 生成洞
    num = 0
    h_num = 0
    holes = []
    while h_num < num_holes:
        x = random.randint(min_x * 2, max_x * 2) * 0.5
        y = random.randint(min_y * 2, max_y * 2) * 0.5
        lx = random.randint(1, 3) * 0.5
        ly = random.randint(1, 3) * 0.5
        x2 = x + lx
        y2 = y + ly
        hole_point = np.array([[x, y], [x2, y], [x2, y2], [x, y2]])
        if Polygon(hole_point).within(fixed_polygons):
            holes.append(hole_point)
            fixed_polygons = fixed_polygons.difference(Polygon(hole_point))
            h_num += 1
        num += 1
        assert num < 100
    return fixed_polygons, vertices_array, holes


def generate_random_polygon_fixed(min_x, max_x, min_y, max_y, num_points, hole_num, **kwargs):
    assert num_points % 2 == 0
    seed = kwargs.get('seed', random.randint(0, 2e9))
    flag = 1
    while flag:
        try:
            fixed_polygons, vertices_array, holes = generate_random_polygon(min_x, max_x, min_y, max_y, num_points, hole_num, seed=seed)
            flag = 0
        except:
            seed = seed + 1
    return fixed_polygons, vertices_array.T, holes, seed
