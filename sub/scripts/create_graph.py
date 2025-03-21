import numpy as np
from inpolygon import inpolygon
import math
import networkx as nx


def create_graph(step, crs, holes):
    # ---------------------------------------------------------------------------#
    # # map: 地图边界点
    # # step: 栅格地图步长
    # # 地图 xy 极值；
    # ---------------------------------------------------------------------------#
    xmin = min(crs[:, 0])
    xmax = max(crs[:, 0])
    ymin = min(crs[:, 1])
    ymax = max(crs[:, 1])
    # 初始栅格；
    x_step = np.arange(xmin, xmax + step, step)
    y_step = np.arange(ymin, ymax + step, step)
    xx, yy = np.meshgrid(x_step, y_step)
    # 偏移栅格；
    xx = xx + step * 0.5
    yy = yy + step * 0.5
    # [0 0.25 0.5 0.75 1] -> [0.125 0.375 0.625 0.875]

    # 在地图内的偏移栅格；
    in_point = inpolygon(xx, yy, crs[:, 0], crs[:, 1], holes)[1]
    xx_in = xx[in_point]
    yy_in = yy[in_point]

    # 记录节点编号和坐标之间的关系
    num_point_in = np.shape(xx_in)[0]
    idx = 1
    idx2coor = np.empty((0,3))
    for i in range(xx_in.shape[0]):
        a = np.array([idx, xx_in[i], yy_in[i]])
        a=a.reshape(1,3)
        idx2coor = np.concatenate((idx2coor, a), axis=0)
        idx = idx + 1

    # 构建邻接矩阵；
    adj = np.ones((num_point_in, num_point_in)) * np.inf
    # 邻接矩阵赋值
    for i in range(xx_in.shape[0]):
        for j in range(xx_in.shape[0]):
            norms = np.array([xx_in[i] - xx_in[j], yy_in[i] - yy_in[j]])
            dis = np.linalg.norm(norms, ord=2)
            # 根据距离判断是否相连接，8邻域为可达点
            if dis < math.sqrt(2) * step * 1.1:
                adj[i, j] = dis / 2
                adj[j, i] = dis / 2
    # ---------------------------------------------------------------------------#
    # 生成无向图
    # ---------------------------------------------------------------------------#
    G = nx.Graph()
    # Add nodes to the graph
    num_nodes = adj.shape[0]
    G.add_nodes_from(range(1, num_nodes + 1))
    # Add edges to the graph
    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            if adj[i, j]!=np.inf:
                G.add_edge(i + 1, j + 1,weight=adj[i, j])
    return G, idx2coor
