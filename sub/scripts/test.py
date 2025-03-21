import time
import numpy as np
from inpolygon import inpolygon
from shapely.geometry import Polygon
from cool import cool
import matplotlib.patches as patches
import matplotlib.pyplot as plt
from create_graph import create_graph
from KOrderVoronoi import KOrderVoronoi
from vis_poly import vis_poly
from find_neighbors import find_neighbors
from find_reflex import find_reflex
from geodesic_distance import geodesic_distance
from density_gaussian import density_gaussian
from pfai_pt import pfai_pt
from find_endpoints_acrs import find_endpoints_acrs
# ---------------------------------------------------------------------------#
# 设置随机数种子
# ---------------------------------------------------------------------------#
np.random.seed(0)

start_time = time.time()
# ---------------------------------------------------------------------------#
# 初始化变量
# ---------------------------------------------------------------------------#
numinterations = 1
showPlot = 1
R = 0.7
k = 2
n = 5
H_old = 0
H = []
xrange = 6.5
yrange = 4.5
dt = 0.005
eps = 1e-10
multi = 2e5
tao = 0.2
ke = 10.0
step = 0.2  # 创建无向图的步长
# idx_T1 = zeros(n, 6) #存储循环次数，节点编号，[x y]，[T1x T1y]
mu = np.array([6,3])
mu=mu.reshape(1,2)
r = mu
once = 0
# crs = [0, 0
# 0, yrange
# xrange, yrange ####凸域
# xrange, 0]
# crs = [0, 0
# 0, yrange
# 1 / 3 * xrange, yrange
# 1 / 3 * xrange, 2 / 5 * yrange ####凹型 \
# 2 / 3 * xrange, 2 / 5 * yrange
# 2 / 3 * xrange, yrange
# xrange, yrange
# xrange, 0]
crs = np.array([[0, 2 / 5 * yrange],  # 1
                [0, yrange],  # 2
                [1 / 4 * xrange, yrange],  # 3
                [1 / 4 * xrange, 4 / 5 * yrange],  # 4
                [9 / 12 * xrange, 4 / 5 * yrange],  # 5
                [9 / 12 * xrange, yrange],  # 6
                [11 / 12 * xrange, yrange],  # 7
                [11 / 12 * xrange, 9 / 10 * yrange],  # 8
                [xrange, 9 / 10 * yrange],  # 9
                [xrange, 11 / 20 * yrange],  # 10
                [9 / 24 * xrange, 11 / 20 * yrange],  # 11
                [9 / 24 * xrange, 4 / 10 * yrange],  # 12
                [11 / 12 * xrange, 4 / 10 * yrange],  # 13
                [11 / 12 * xrange, 2 / 15 * yrange],  # 14
                [1 / 2 * xrange, 2 / 15 * yrange],  # 15
                [1 / 2 * xrange, 0],  # 16
                [1 / 6 * xrange, 0],  # 17
                [1 / 6 * xrange, 2 / 5 * yrange],  # 18
                [0, 2 / 5 * yrange]])
# crs = np.array([[0, 0],  # 1
#                 [0, yrange],  # 2
#                 [xrange, yrange],  # 3
#                 [xrange, 0],  # 4
#                ])  # 18
# 初始位置（列向量）
# xi = np.random.random(size=(n,0)) * xrange * 0.1 + 1
# yi = np.random.random(size=(n,0))* yrange * 0.1
xi=np.array(([1],[2],[1.5],[2],[3]))
yi=np.array(([2],[1],[2],[3],[3]))

# for i in range(xi.shape[0]):
#     while not inpolygon(xi[i],yi[i],crs[:,0],crs[:,1])[1] :
#         xi[i] = np.random.rand([1, 1]) * xrange * 0.1 + 1
#         yi[i] = np.random.rand([1, 1]) * yrange * 0.1
x_in = xi
y_in = yi
poly = Polygon(crs)
#可视化非凸区域
# plt.plot(*poly.exterior.xy)
# plt.show()
# ---------------------------------------------------------------------------#
# VISUALIZATION
# ---------------------------------------------------------------------------#
# if (showPlot):
#     verCellHandle = np.zeros((n,1))
#     cellColors = cool(n)
#     for i in range(len(x_in)):  # color according to
#         verCellHandle[i] = patches(x_in[i], y_in[i], cellColors(i,:), 'FaceAlpha', 0.2)  # use color i - - no robot assigned yet
#     pathHandle = np.zeros((n, 1))
#     numHandle = np.zeros((n, 1))
#     for i in range(len(x_in)):  # color according to
#         pathHandle[i] = plt.plot(x_in[i], y_in[i], '-', color=cellColors[i,:]*.8)
#     goalHandle = plt.plot(x_in, y_in, '+', 'linewidth', 2)
#     currHandle = plot(x_in, y_in, 'o', 'linewidth', 2)
#     titleHandle = title(['o = Robots, + = Initials, Iteration ', num2str(0)])


# 创建当前环境的无向图
G, idx2coor = create_graph(step, crs)

for counter in range(numinterations):
    # print(f'Iter: {counter}, Total time usage: {(time.time() - start_time) / 1e7}\n')
    # start_time_part1 = time.time()
    # # 计算非凸域的高阶维诺划分
    # vi, ci = KOrderVoronoi(crs, x_in, y_in, R, k)
    # VV = vi
    # C = ci
    # print(f'KOrderVoronoi calculation: {(time.time() - start_time_part1)/ 1e7}\n' )
    # start_time_part2 = time.time()
    # # 计算可见多边形
    # [Vpolyi, Cpolyi] = vis_poly(crs, x_in, y_in, R,poly)
    # Vpoly = Vpolyi
    # Cpoly = Cpolyi
    # print(f'vis_poly calculation: {(time.time() - start_time_part2) / 1e7}\n')
    # start_time_part3 = time.time()
    # # 标出当前目标区域
    # # showmu = plt.scatter(mu[:, 0], mu[:, 1], s=30, color='r')
    #
    # ##################求高阶维诺和可见多边形的交集 ##########################################
    # D = np.zeros((0, 2))
    # Cd = []
    # for ij in range(len(x_in)):
    #     bound = Polygon(VV[C[ij]])
    #     Vps = Polygon(Vpoly[Cpoly[ij]])
    #     polyout = bound.intersection(Vps)
    #     boundsetij = list(polyout.exterior.coords)
    #     boundsetij=np.array(boundsetij)
    #     if len(boundsetij)!=0:
    #         Cd.append(np.arange(D.shape[0], D.shape[0] + boundsetij.shape[0]))
    #         D=np.concatenate((D, boundsetij))
    #     else:
    #         Cd.append(None)
    # Vn,Cn=find_neighbors(x_in, y_in, R, D, Cd)
    # Pr,Cr=find_reflex(D,Cd,crs,x_in,y_in,R)
    # x_1,y_1,d=geodesic_distance(G, idx2coor, x_in, y_in, mu)
    # start_point, end_point =find_endpoints_acrs(D[Cd[0]], R)
    fai=density_gaussian(mu, x_in, y_in)
    # fai_pt=pfai_pt(x_in, y_in, mu, r, ke, fai)
    print(fai)
# ---------------------------------------------------------------------------#
# 可视化部分
# ---------------------------------------------------------------------------#
# plt.scatter(x_in,y_in,s=10)
# plt.plot(*poly.exterior.xy)
# for i in range(len(x_in)):
#     plt.plot(D[Cd[i], 0], D[Cd[i], 1])
# for i in range(len(x_in)):
#     plt.text(x_in[i]*1.01, y_in[i]*1.01, str(i), fontsize=10) #给散点加标签
# plt.show()