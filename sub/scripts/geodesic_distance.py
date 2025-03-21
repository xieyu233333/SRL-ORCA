import time

import numpy as np
import networkx as nx
from scipy.spatial import distance
from shapely import LineString

def geodesic_distance(G, idx2coor, x, y, mu):
    # GEODESIC_DISTANCE 根据测地线距离转化坐标
    d = np.zeros((0,))
    dist_matrix = distance.cdist(np.concatenate((x[:,np.newaxis], y[:,np.newaxis]), axis=1), idx2coor[:, 1:])
    ki = np.argmin(dist_matrix, axis=1)
    dist_matrix2 = distance.cdist(mu, idx2coor[:, 1:])

    kmu = np.argmin(dist_matrix2, axis=1)
    start_time = time.perf_counter()
    for i in range(len(ki)):
        source = (idx2coor[ki[i], 0]).astype(int)
        target = (idx2coor[kmu[0], 0]).astype(int)
        shortest_distance = nx.dijkstra_path_length(G, source, target)
        d = np.append(d, np.array([shortest_distance]), axis=0)
    print(time.perf_counter() - start_time)
    x_1 = mu[:, 0] * np.ones_like(x)
    y_1 = -d + mu[:, 1]
    x_1 = x_1[:, np.newaxis]
    y_1 = y_1[:, np.newaxis]
    return x_1, y_1


def geodesic_distance_new(result_dict, idx2coor, pts_1, pt_2):
    """
    计算最短距离
    :param result_dict: 图的距离计算结果
    :param idx2coor: 索引
    :param pts_1: 点1（可以是1组），表示为[[x1, y1], ..., [xi, yi]] (np.array)
    :param pt_2: 点2（1个）
    :return: 用最短距离算出来的一个等价点，用于后续的密度函数计算
    """
    # TODO: pt_2改成多个点
    # dist_matrix = np.sqrt(np.sum((pts_1[:, np.newaxis, :] - idx2coor[np.newaxis, :, 1:]) ** 2, axis=2))
    dist_matrix = distance.cdist(pts_1, idx2coor[:, 1:])
    ki = np.argmin(dist_matrix, axis=1)
    # dist_matrix2 = np.sqrt(np.sum((pt_2[:, np.newaxis, :] - idx2coor[np.newaxis, :, 1:]) ** 2, axis=2))
    dist_matrix2 = distance.cdist(pt_2, idx2coor[:, 1:])
    kmu = np.argmin(dist_matrix2, axis=1)[0]
    d = np.array([result_dict[ki[i]+1][kmu] for i in range(len(ki))])
    x_1 = pt_2[:, 0] * np.ones_like(pts_1[:, 0])
    y_1 = -d + pt_2[:, 1]
    x_1 = x_1[:, np.newaxis]
    y_1 = y_1[:, np.newaxis]
    return x_1, y_1

