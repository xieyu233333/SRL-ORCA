import time
from KOrderVoronoi import KOrderVoronoi, KOrderVoronoi_poly
from vis_poly import vis_poly
from publisher import pub
from find_reflex import find_reflex
from control_law import control_law
import numpy as np
from shapely.geometry import Polygon, Point, LineString


class Calc:
    def __init__(self, crs, holes, xi, yi, yaw_i, cmd_vel_pub,mu, ri,result_dict, idx2coor, **kwargs):
        self.crs = crs
        self.holes = holes
        self.poly = Polygon(crs)
        self.x_in = xi.copy()
        self.y_in = yi.copy()
        self.yaw_ori = yaw_i.copy()
        self.xi = xi
        self.yi = yi
        self.ri = ri
        self.yaw_i = yaw_i
        self.cmd_vel_pub = cmd_vel_pub
        self.r = kwargs.get('mu', np.array([[5.5, 4]]))
        self.R = kwargs.get('R', 0.7)
        self.k = kwargs.get('k', 2)
        self.result_dict = result_dict
        self.idx2coor = idx2coor
        self.mu = mu
        self.eps = kwargs.get('eps', 1e-10)
        self.multi = kwargs.get('multi', 2e5)
        self.ke = kwargs.get('ke', 10.0)
        self.dt = kwargs.get('dt', 0.005)
        self.seed = kwargs.get('seed', None)
        self.xrange = kwargs.get('xrange', 6.5)
        self.yrange = kwargs.get('yrange', 4.5)
        self.counter = 0
        # self.file = open("output.txt", "w")


    def calculate(self, plt_vis):
        self.counter += 1
        start_time_part1 = time.perf_counter()
        # 计算非凸域的高阶维诺划分
        vi, ci = KOrderVoronoi(self.crs, self.holes, self.x_in, self.y_in, self.R, self.k)
        VV = vi
        C = ci
        # print(f'KOrderVoronoi calculation: {(time.perf_counter() - start_time_part1) / 1e0}\n')
        start_time_part2 = time.perf_counter()
        # 计算可见多边形
        [Vpolyi, Cpolyi] = vis_poly(self.crs, self.holes, self.x_in, self.y_in, self.R, self.poly)
        Vpoly = Vpolyi
        Cpoly = Cpolyi
        # print(f'vis_poly calculation: {(time.perf_counter() - start_time_part2) / 1e0}\n')
        start_time_part3 = time.perf_counter()
        # ---------------------------------------------------------------------------#
        # 求高阶维诺和可见多边形的交集
        # ---------------------------------------------------------------------------#
        D = np.zeros((0, 2))
        Cd = []
        for ij in range(len(self.x_in)):
            bound = Polygon(VV[C[ij]])
            Vps = Polygon(Vpoly[Cpoly[ij]])
            polyout = bound.intersection(Vps)
            boundsetij = list(polyout.exterior.coords)
            boundsetij = np.array(boundsetij)
            if len(boundsetij) != 0:
                Cd.append(np.arange(D.shape[0], D.shape[0] + boundsetij.shape[0]))
                D = np.concatenate((D, boundsetij))
            else:
                Cd.append(None)
        # ---------------------------------------------------------------------------#
        # 控制率更新
        # ---------------------------------------------------------------------------#
        Pr, Cr = find_reflex(D, Cd, self.crs, self.holes, self.x_in, self.y_in, self.R)
        print(f'part-3 calculation: {(time.perf_counter() - start_time_part3) / 1e0}\n')
        start_time_part4 = time.perf_counter()
        for ij in range(len(self.x_in)):
            Di = D[Cd[ij], :]
            [ui] = control_law(self.result_dict, self.idx2coor, Di, self.x_in, self.y_in, ij, self.R, self.mu,
                               Pr, Cr, self.eps, self.multi, self.r, self.ke)
            # print(f'Agent {ij + 1}, ui_original = [{ui[0]}, {ui[1]}]\n')
            # fprintf('Agent #d, ui = [#.4f, #.4f]\n', ij, ui(1), ui(2))
            # self.x_in[ij] = self.x_in[ij] + self.dt * ui[0]
            # self.y_in[ij] = self.y_in[ij] + self.dt * ui[1]
            ui = self.dt * ui
            # print(f'Agent {ij + 1}, ui_now = [{ui[0]}, {ui[1]}]\n')
            pub(ui, self.yaw_ori[ij], ij, self.cmd_vel_pub)
        # print(f'ui calculation: {(time.perf_counter() - start_time_part4) / 1e0}\n')
        self.x_in = self.xi.copy()
        self.y_in = self.yi.copy()
        self.yaw_ori = self.yaw_i.copy()
        # print('yaw2:',self.yaw_ori)
        # ---------------------------------------------------------------------------#
        # 更新Mu
        # ---------------------------------------------------------------------------#
        self.r = self.ri.copy()
        if np.all(self.r ==np.zeros((1,2))):
            self.r=self.mu
        print('r=',self.r)
        self.mu = self.mu - self.ke * (self.mu - self.r) * self.dt
        plt_vis.update_para(D, Cd, self.x_in, self.y_in, self.mu)

