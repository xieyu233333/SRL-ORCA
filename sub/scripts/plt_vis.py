import time

import matplotlib.pyplot as plt
import numpy as np
from matplotlib.animation import FuncAnimation
import matplotlib.animation as ani
from shapely.geometry import Polygon
from matplotlib.path import Path
from matplotlib.patches import PathPatch

class PlotVis:
    def __init__(self, calc, **kwargs):
        self.linedata = []
        self.D = None
        self.Cd = None
        self.poly = calc.poly
        self.x_in = None
        self.y_in = None
        self.mu = None
        self.animation = None
        self.calc = calc
        self.calc_time = []
        self.path1 = None
        self.path2 = None
        self.crs = calc.crs
        self.holes = calc.holes
        self.seed = calc.seed
        self.xrange = calc.xrange
        self.yrange = calc.yrange
        self.savefig = kwargs.get('savefig', False)

    def update_para(self, D, Cd, x_in, y_in, mu):
        self.D = D
        self.Cd = Cd
        self.x_in = x_in
        self.y_in = y_in
        self.mu = mu

    def cal(self):
        self.calc.calculate(self)
        self.calc_time.append(time.perf_counter())

    def plt_env(self):
        fig, ax = plt.subplots()
        self.create_path()
        l1 = ax.add_patch(PathPatch(self.path1, fc ="none"))
        l2 = ax.add_patch(PathPatch(self.path2, fc="black"))

        # ax.scatter(np.array([[0.71991086],[1.87127512],[1.88349436],[3.04631418],[1.28544089]]), np.array([[2.39218809],[0.65061375],[1.8056948 ],[1.8002136 ],[0.64435691]]), s=10, c='k')

        ax.title.set_text('Seed: {}, Iter: {} ({} Hz)'.format(self.seed, self.calc.counter, self.hz()))
        ax.set_xlim(-1, self.xrange + 1)
        ax.set_ylim(-1, self.yrange + 1)
        plt.show()

    def hz(self):
        return np.sum(abs(time.perf_counter() - np.array(self.calc_time)) < 1).astype(int)

    def create_path(self):
        vertices = np.vstack((self.crs[:, 0], self.crs[:, 1])).T
        # 定义Path对象
        vertices_holes = np.zeros((0, 2))
        # 定义codes对象
        codes = []
        for v in self.holes:
            vertices_holes = np.concatenate((vertices_holes, v, np.zeros((1, 2))))
            codes = codes + [Path.MOVETO] + [Path.LINETO] * (v.shape[0] - 1) + [Path.CLOSEPOLY]
        self.path1 = Path(vertices)
        self.path2 = Path(vertices_holes, codes)

    def update_plt(self, _):
        self.cal()

        # self.linedata[0].collections.clear()
        self.linedata[0].collections[1].remove()
        self.linedata[0].collections[0].remove()
        self.linedata[0].title.set_text('Seed: {}, Iter: {} ({} Hz)'.format(self.seed, self.calc.counter, self.hz()))
        self.linedata[3] = self.linedata[0].scatter(self.x_in, self.y_in, s=10, c='k')
        self.linedata[4] = self.linedata[0].scatter(self.mu[:, 0], self.mu[:, 1], s=10, c='r', marker='*')
        for i in range(len(self.x_in)):
            self.linedata[5][i][0].set_data(self.D[self.Cd[i], 0], self.D[self.Cd[i], 1])
        for i in range(len(self.x_in)):
            self.linedata[6][i].set_x(self.x_in[i] * 1.01)
            self.linedata[6][i].set_y(self.y_in[i] * 1.01)

    def plt_vis(self):
        fig, ax = plt.subplots()
        self.create_path()
        l1 = ax.add_patch(PathPatch(self.path1, fc ="none"))
        l2 = ax.add_patch(PathPatch(self.path2, fc="black"))
        # l1 = ax.plot(*self.poly.exterior.xy)
        l3 = ax.scatter(self.x_in, self.y_in, s=10, c='k')
        l4 = ax.scatter(self.mu[:, 0], self.mu[:, 1], s=10, c='r', marker='*')
        l5 = []
        for i in range(len(self.x_in)):
            l5.append(ax.plot(self.D[self.Cd[i], 0], self.D[self.Cd[i], 1]))
        l6 = []
        for i in range(len(self.x_in)):
            l6.append(ax.text(self.x_in[i] * 1.01, self.y_in[i] * 1.01, str(i+1), fontsize=10))  # 给散点加标签
        ax.title.set_text('Seed: {}, Iter: {} ({} Hz)'.format(self.seed, self.calc.counter, self.hz()))
        self.linedata = [ax, l1, l2, l3, l4, l5, l6]
        self.animation = FuncAnimation(fig, self.update_plt, frames=1000, interval=5)
        plt.show()
        if self.savefig and False:
            FFwriter = ani.FFMpegWriter(fps=10)
            self.animation.save('animation.mp4', writer=FFwriter)


# def plt_vis(D, Cd, poly, x_in, y_in, mu, fig):
#     polygons = []
#     for i in range(len(x_in)):
#         polygons.append(D[Cd[i]])
#     plt.ion()  ## Note this correction
#     ax = fig.add_subplot(111)
#     ax.set_xticks([])
#     ax.set_yticks([])
#     # ax.set_aspect('equal')
#     # ax.set_xlim(bounds[0], bounds[1])
#     # ax.set_ylim(bounds[2], bounds[3])
#     robot_handle = ax.scatter(x_in, y_in, 20, 'black')
#     # centroid_handle = ax.scatter(centroids[:, 0], centroids[:, 1], 20, marker='s',
#     #                                        color='green')
#
#     polygon_handle = []
#     for ii in range(len(polygons)):
#         polygon_handle.append(
#             ax.fill(*zip(*polygons[ii]), alpha=0.1, edgecolor='black', linewidth=3))
#
#     # UPDATE VORONOI CELLS
#     [p.remove() for p in reversed(ax.patches)]
#     for ii in range(len(polygons)):
#         polygon_handle[ii] = ax.fill(*zip(*polygons[ii]), alpha=0.1, edgecolor='black', linewidth=3)
#
#     # UPDATE ROBOT AND CENTROID
#     xy = [[x_in[i], y_in[i]] for i in range(len(x_in))]
#     robot_handle.set_offsets(xy)
#     # centroid_handle.set_offsets(centroids)
#     fig.canvas.draw()
#     fig.canvas.flush_events()
#     plt.show()