# coding=utf8
import time

import numpy as np
import shapely
from shapely import Point
from matplotlib.path import Path


def inpolygon(xq, yq, xv, yv, holes=None):
    """
    reimplement inpolygon in matlab
    :param holes: a list of np.ndarray
    :type xq: np.ndarray
    :type yq: np.ndarray
    :type xv: np.ndarray
    :type yv: np.ndarray
    """
    # 合并xv和yv为顶点数组
    if holes is None:
        holes = []
    vertices = np.vstack((xv, yv)).T
    vertices_hole = np.zeros((0, 2))
    # 定义codes对象
    codes = []
    for v in holes:
        vertices_hole = np.concatenate((vertices_hole, v, np.zeros((1, 2))))
        codes = codes + [Path.MOVETO] + [Path.LINETO] * (v.shape[0] - 1) + [Path.CLOSEPOLY]
    path1 = Path(vertices)
    path2 = Path(vertices_hole, codes)
    # 把xq和yq合并为test_points
    test_points = np.hstack([xq.reshape(xq.size, -1), yq.reshape(yq.size, -1)])
    # 得到一个test_points是否严格在path内的mask，是bool值数组
    _in_1 = path1.contains_points(test_points)
    # 得到一个test_points是否在path内部或者在路径上的mask
    _in_on_1 = path1.contains_points(test_points, radius=-1e-10)
    _in_2 = path2.contains_points(test_points)
    _in_on_2 = path2.contains_points(test_points, radius=-1e-10)
    # 得到一个test_points是否在path路径上的mask
    # _on_1 = _in_1 ^ _in_on_1
    _in_on = np.logical_not(np.logical_or(np.logical_not(_in_on_1), _in_on_2))
    _in = np.logical_not(np.logical_or(np.logical_not(_in_1), _in_2))
    _in_on=_in_on.reshape(xq.shape)
    # _on=_on.reshape(xq.shape)
    _in = _in.reshape(xq.shape)
    return _in_on, _in


def inpolygon_new(poly, pts_x, pts_y):
    test_points = np.hstack([pts_x.reshape(pts_x.size, -1), pts_y.reshape(pts_y.size, -1)])
    s = time.perf_counter()
    test_points = np.apply_along_axis(lambda p: Point(p[0], p[1]), axis=0, arr=test_points.T)  # very slow
    print(time.perf_counter() - s)
    result = shapely.contains(poly, test_points)
    result = result.reshape(pts_x.shape)
    return result, np.inf  # 只是占位一下

# if __name__ == '__main__':
#     xv = np.array([-4, 0, 4, 0])
#     yv = np.array([0, 4, 0, -4])
#     X = np.array([0, 1, 3.5, 4, 5])
#     Y = np.array([0, 1, 0,   0, 0])
#
#     _in, _on = inpolygon(X, Y, xv, yv)
#     print (_in, _on)
