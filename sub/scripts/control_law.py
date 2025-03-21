import time

from geodesic_distance import geodesic_distance_new
from density_gaussian import density_gaussian
from find_endpoints_acrs import find_endpoints_acrs
from pfai_pt import pfai_pt
import numpy as np
def control_law(result_dict, idx2coor, Di, x_in, y_in, i, R, mu, Pr, Cr, eps, multi, r, ke):
    # ---------------------------------------------------------------------------#
    # 根据控制率计算控制输入u
    # --------------------------------------------------------------------------#
    T1 = np.zeros((0, 2))  # 相交边界圆弧上no和phi积分
    T2 = np.zeros((0, 2))  # Ps上的积分
    T1_x = np.zeros((0, 2))
    T1_t = np.zeros((0, 2))
    T2_x = np.zeros((0, 2))
    T2_t = np.zeros((0, 2))
    b = np.zeros((0, 2))
    Ci = []  # 存储相交圆弧的索引
    Dci = np.zeros((0, 2))  # 存储相交圆弧的点坐标
    # 判断管辖区域的边界，哪些点是圆弧上的，判断方式是计算边界点离中心的距离，距离=R则认为该点落于圆弧上
    di = Di - np.concatenate((x_in[i][:,np.newaxis], y_in[i][:,np.newaxis]), axis=1)
    diffs = abs(np.linalg.norm(di, axis=1) - R)
    Ci = np.where(diffs < 0.0001)  # 找到在i的传感圆盘上的索引
    # for p in range(len(neighs)):  # 遍历每个邻居节点，找到Di和j传感圆盘的交集
    #     xn = x_in[neighs[p]]
    #     yn = y_in[neighs[p]]
    #     dn = Di - np.concatenate((xn[:,np.newaxis], yn[:,np.newaxis]), axis=1)
    #     diffs = abs(np.linalg.norm(dn, axis=1) - R)
    #     cn = np.where(diffs < 0.0001)
    #     if cn:
    #         Ci.append(cn)

    if np.array(Ci).shape[1] != 0:
        Dci = np.squeeze(Di[Ci, :])  # Di圆弧上所有的点
        n0 = (Dci - np.array([x_in[i], y_in[i]]).T) / R
        # Dci=np.squeeze(Dci)
        # g=np.gradient(Dci.T,axis=1)
        # g=g.T
        # g=g/np.linalg.norm(g,axis=1)[:,np.newaxis]
        # n0=np.concatenate((-g[:,1:],g[:,0:1]),axis=1)
        # norms = np.linalg.norm(Dci - np.concatenate((x_in[i][:,np.newaxis], y_in[i][:,np.newaxis]), axis=1), axis=1)
        # n0 = (Dci - np.concatenate((x_in[i][:,np.newaxis], y_in[i][:,np.newaxis]), axis=1)) / norms[:, np.newaxis]  # 圆弧上单位法向量
        # x_1, y_1 = geodesic_distance(G, idx2coor, Dci[:, 0], Dci[:, 1], mu)
        x_1, y_1 = geodesic_distance_new(result_dict, idx2coor, Dci, mu)
        fai = density_gaussian(mu, x_1, y_1)  # 圆弧上的fai
        T1 = np.nansum(multi * (fai + eps) * n0, axis=0).reshape(1, -1) # 对圆弧上的点fai * 单位法向量再积分

        start_point, end_point = find_endpoints_acrs(Dci, R)
        if len(start_point) != 0 and len(end_point) != 0:
            b = -multi * (fai[start_point] + eps) * n0[start_point, :] + multi * (fai[end_point] + eps) * n0[end_point,:]
        else:
            b = 0
        T1_x = np.nansum(b, axis=0).reshape(1, -1) + np.nansum(multi * (fai + eps), axis=0).reshape(1, -1) / R
        fai_pt = pfai_pt(Dci[:, 0], Dci[:, 1], mu, r, ke, fai)
        T1_t = np.nansum(multi * fai_pt * n0, axis=0).reshape(1, -1)

    Pri = Pr[Cr[i], :]  # 存储i的非凸点

    for ij in range(Pri.shape[0]):  # 一个Di可能有多个非凸点
        xy_in = np.concatenate((x_in[i][:,np.newaxis], y_in[i][:,np.newaxis]), axis=1)
        dist = Pri[ij, :] - xy_in
        detamax = abs(R - np.linalg.norm(dist, axis=1))
        np_1 = dist / np.linalg.norm(dist, axis=1)  # 计算Pri和xi的单位向量
        q = []
        # g_x,g_y=np.gradient(dist.T)
        # g_y=g_y.T
        # g_y=g_y/np.linalg.norm(g_y,axis=1)[:,np.newaxis]
        # no_p=np.concatenate((-g_y[:,1:],g_y[:,0:1]),axis=1)
        no_p = np.concatenate((-np_1[:,1][:,np.newaxis], np_1[:,0][:,np.newaxis]), axis=1)  # Pri和xi的法向量
        deta = np.arange(0,detamax,0.01)[:,np.newaxis]
        q = Pri[ij,:].reshape(1,2)+ np_1 * deta
        # x_1, y_1 = geodesic_distance(G, idx2coor, q[:, 0], q[:, 1], mu)
        x_1, y_1 = geodesic_distance_new(result_dict, idx2coor, q, mu)
        fai = density_gaussian(mu, x_1, y_1)
        inte1 = np.nansum(multi * (fai + eps) * deta, axis=0).reshape(1, -1)
        T2 = T2 + no_p / np.linalg.norm(dist)* inte1
        fai_pt = pfai_pt(q[:, 0], q[:, 1], mu, r, ke, fai)
        inte2 = np.nansum(multi * fai_pt * deta, axis=0).reshape(1, -1)
        T2_t = T2_t + no_p / np.linalg.norm(dist) * inte2
        T2_x = T2_x + (no_p / (np_1 * (np.linalg.norm(dist))**2)) * inte1
        #

    if len(T2)==0:
        T2=np.array([[0,0]])
    if len(T2_x)==0:
        T2_x = np.array([[0, 0]])
    if len(T1_t)==0:
        T1_t =np.array([[0,0]])
    if len(T2_t)==0:
        T2_t = np.array([[0, 0]])

    ui = (T2_t - T1_t + 1e3 * (T1 - T2))/ (T1_x - T2_x)
    # print(f'Agent:{i+1}, 达到构型\n')
    # ui = T1 - T2
    # print(f'Agent:{i+1}, 未达到构型，norm = {np.linalg.norm(ui)}\n')

    if np.linalg.norm(ui) < 0.1:
        ui = ui * 13.0
    if np.linalg.norm(ui) == 0:
        ui = np.array([[0, 0]])
    elif np.linalg.norm(ui) > 30.0:
        ui = ui / np.linalg.norm(ui) * 30.0
    return ui
