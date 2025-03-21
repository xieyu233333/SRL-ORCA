import numpy as np
import math
from inpolygon import inpolygon, inpolygon_new


def KOrderVoronoi(crs, holes, x_in, y_in, R, k):
    """
    计算k阶维诺图
    输入集群初始位置，地图信息，传感半径R，维诺图阶次k；输出维诺单元边界坐标
    
    :param x_in: 
    :param y_in: 
    :param R: 
    :param k: 
    :param crs: 
    :return: 
    """
    VV = np.zeros((0, 2))
    C = []
    for p in range(x_in.shape[0]):  # 节点遍历
        rho = 0.02  # 半径变化步长
        theta = 0.1  # 角度变化量

        ###########计算第i个节点的边界
        xs = np.concatenate((x_in[:p], x_in[p + 1:]), axis=0)  # 存储除了i以外节点的坐标（数组）
        ys = np.concatenate((y_in[:p], y_in[p + 1:]), axis=0)
        x = []
        y = []
        xn = x_in[p]  # 存储i节点的坐标（数）
        yn = y_in[p]
        # 找出通信范围内的邻居节点
        l_xs = len(xs)
        for n in range(l_xs):
            dd = math.sqrt((xs[n] - xn) ** 2 + (ys[n] - yn) ** 2)
            if dd <= 2 * R and inpolygon(xs[n], ys[n], crs[:, 0], crs[:, 1], holes)[0]:
                x.append(xs[n])  # 存储i的所有邻居节点（数组）
                y.append(ys[n])
        l_x = len(x)
        # ---------------------------------------------------------------------------#
        # ##以当前节点i为中心，rho为半径，角度步长为0.01度，画圆圈
        #     #计算思路：
        #     #1.
        #     首先生成所有节点
        #     #2.
        #     判断哪些节点在域内
        #     #3.
        #     判断域内每个节点离通信区域的节点的距离
        #     #4.
        #     计数，通信数量，条件2的数量会被x100，然后和条件3的累加
        #     #5.
        #     找到临界值，即每个直线上的点在何时超过某个阈值
        #     #6.
        #     记录临界值序号
        #     #7.
        #     找到对应的x
        #     y坐标，存入对应变量
        # ---------------------------------------------------------------------------#
        # 1.
        r_i = np.arange(0, R + rho, rho)
        w_i = np.arange(0, 360 + theta, theta)[0::10]
        w_i = np.radians(w_i)
        r_i_T = np.transpose([r_i])
        pts_x = xn + r_i_T @ np.cos(w_i)[np.newaxis, :]
        pts_y = yn + r_i_T @ np.sin(w_i)[np.newaxis, :]
        # 2.
        result_poly = inpolygon(pts_x, pts_y, crs[:, 0], crs[:, 1], holes)[0]
        # 3. + 4.
        M0 = 100

        result_count = ((1 - result_poly.astype(bool)).astype(int)) * M0
        dist_std = r_i_T * np.ones((1, len(w_i)))
        for j in range(l_x):
            # 3.
            dist_mat = np.sqrt((x[j] - pts_x) ** 2 + (y[j] - pts_y) ** 2)
            # 4.
            result_count += (dist_mat < dist_std)
        # 5.
        val = np.max((result_count >= k).astype(int), axis=0)
        idx = np.maximum(np.argmax(result_count >= k, axis=0) - 1, 0)

        # 6.
        max_idx = len(r_i) - 1
        idx = idx * val + max_idx * (1 - val)
        # 7.
        boundsetxi = pts_x[idx, np.arange(idx.shape[0])]
        boundsetyi = pts_y[idx, np.arange(idx.shape[0])]

        boundseti = np.concatenate((boundsetxi[:, np.newaxis], boundsetyi[:, np.newaxis]), axis=1)
        # boundseti = boundseti[0::10]
        C.append(np.arange(VV.shape[0], VV.shape[0] + boundseti.shape[0]))
        VV = np.concatenate((VV, boundseti))

    return VV, C




def KOrderVoronoi_poly(poly, x_in, y_in, R, k):
    """
    计算k阶维诺图
    输入集群初始位置，地图信息，传感半径R，维诺图阶次k；输出维诺单元边界坐标

    :param x_in:
    :param y_in:
    :param R:
    :param k:
    :param crs:
    :return:
    """
    VV = np.zeros((0, 2))
    C = []
    for p in range(x_in.shape[0]):  # 节点遍历
        rho = 0.02  # 半径变化步长
        theta = 0.1  # 角度变化量

        ###########计算第i个节点的边界
        xs = np.concatenate((x_in[:p], x_in[p + 1:]), axis=0)  # 存储除了i以外节点的坐标（数组）
        ys = np.concatenate((y_in[:p], y_in[p + 1:]), axis=0)
        x = []
        y = []
        xn = x_in[p]  # 存储i节点的坐标（数）
        yn = y_in[p]
        # 找出通信范围内的邻居节点
        l_xs = len(xs)
        for n in range(l_xs):
            dd = math.sqrt((xs[n] - xn) ** 2 + (ys[n] - yn) ** 2)
            if dd <= 2 * R and inpolygon_new(poly, xs[n], ys[n])[0]:
                x.append(xs[n])  # 存储i的所有邻居节点（数组）
                y.append(ys[n])
        l_x = len(x)
        # ---------------------------------------------------------------------------#
        # ##以当前节点i为中心，rho为半径，角度步长为0.01度，画圆圈
        #     #计算思路：
        #     #1.
        #     首先生成所有节点
        #     #2.
        #     判断哪些节点在域内
        #     #3.
        #     判断域内每个节点离通信区域的节点的距离
        #     #4.
        #     计数，通信数量，条件2的数量会被x100，然后和条件3的累加
        #     #5.
        #     找到临界值，即每个直线上的点在何时超过某个阈值
        #     #6.
        #     记录临界值序号
        #     #7.
        #     找到对应的x
        #     y坐标，存入对应变量
        # ---------------------------------------------------------------------------#
        # 1.
        r_i = np.arange(0, R + rho, rho)
        w_i = np.arange(0, 360 + theta, theta)[0::10]
        w_i = np.radians(w_i)
        r_i_T = np.transpose([r_i])
        pts_x = xn + r_i_T @ np.cos(w_i)[np.newaxis, :]
        pts_y = yn + r_i_T @ np.sin(w_i)[np.newaxis, :]
        # 2.
        result_poly = inpolygon_new(poly, pts_x, pts_y)[0]
        # 3. + 4.
        M0 = 100

        result_count = ((1 - result_poly.astype(bool)).astype(int)) * M0
        dist_std = r_i_T * np.ones((1, len(w_i)))
        for j in range(l_x):
            # 3.
            dist_mat = np.sqrt((x[j] - pts_x) ** 2 + (y[j] - pts_y) ** 2)
            # 4.
            result_count += (dist_mat < dist_std)
        # 5.
        val = np.max((result_count >= k).astype(int), axis=0)
        idx = np.maximum(np.argmax(result_count >= k, axis=0) - 1, 0)

        # 6.
        max_idx = len(r_i) - 1
        idx = idx * val + max_idx * (1 - val)
        # 7.
        boundsetxi = pts_x[idx, np.arange(idx.shape[0])]
        boundsetyi = pts_y[idx, np.arange(idx.shape[0])]

        boundseti = np.concatenate((boundsetxi[:, np.newaxis], boundsetyi[:, np.newaxis]), axis=1)
        # boundseti = boundseti[0::10]
        C.append(np.arange(VV.shape[0], VV.shape[0] + boundseti.shape[0]))
        VV = np.concatenate((VV, boundseti))

    return VV, C
