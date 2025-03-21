#!/usr/bin/env python3
import sys
sys.path.append("/home/qjm/orca/turtlebot3/src/sub/scripts")
import time,threading,socket
import networkx as nx
import numpy as np
import datetime
from matplotlib import pyplot as plt
from inpolygon import inpolygon
from shapely.geometry import Polygon, Point, LineString,MultiPolygon
from create_graph import create_graph
from KOrderVoronoi import KOrderVoronoi
from vis_poly import vis_poly
from find_neighbors import find_neighbors
from find_reflex import find_reflex
from control_law import control_law
from plt_vis import PlotVis
from calculation import Calc
from publisher import pub,publisher
from subscriber import listener
# from subscriber_dongbu import listener


def tcpserver(ri):
    # 创建socket
    tcp_server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

    # 本地信息
    address = ('192.168.1.27', 5001)

    # 绑定
    tcp_server_socket.bind(address)

    tcp_server_socket.listen(128)

    while True:
        # 等待新的客户端连接
        client_socket, clientAddr = tcp_server_socket.accept()
        while True:
            # time.sleep(0.5)
            # 接收对方发送过来的数据
            recv_data = client_socket.recv(1024)  # 接收1024个字节
            # print('recving')
            if recv_data:
                data=recv_data.decode('gbk')
                split_data=data.split(",")
                if split_data[0].replace('.', '').isdigit():
                    if split_data[1].replace('.', '').isdigit():
                        x = float(split_data[0]) * 20 - 2.897
                        y = float(split_data[1]) * 20
                        if x>=0 and x<=5.0 and y>=0 and y<=4.2:
                            ri[0][0] = x
                            ri[0][1] = y
                            # print('接收到的数据为:x=', x , "y=" , y )
                            print('接收到的数据为:r=', ri)
            else:
                break
        client_socket.close()
    tcp_server_socket.close()
def main():
    # ---------------------------------------------------------------------------#
    # 设置随机数种子
    # ---------------------------------------------------------------------------#
    np.random.seed(0)
    start_time = time.perf_counter()
    # ---------------------------------------------------------------------------#
    # 初始化变量
    # ---------------------------------------------------------------------------#
    R = 0.4
    k = 2
    n = 5
    dt = 0.005
    eps = 1e-10
    multi = 2e5
    ke = 2.0
    step = 0.2 # 创建无向图的步长
    mu = np.array([[3.5, 3]])
    r = mu
    # ---------------------------------------------------------------------------#
    # 接收眼动信息
    # ---------------------------------------------------------------------------#
    ri=np.zeros((1,2))
    # server_thread = threading.Thread(target=tcpserver, args=(ri,))
    # server_thread.start()
    # ---------------------------------------------------------------------------#
    # 初始化位置，自定义
    # ---------------------------------------------------------------------------#
    # xi = np.array(([1], [2], [2.5], [2], [3]), dtype=float)
    # yi = np.array(([2], [1], [1.6], [3], [3]), dtype=float)
    # ---------------------------------------------------------------------------#
    # 初始化位置，从ROS获取
    # ---------------------------------------------------------------------------#
    xi=np.zeros((5,1))
    yi=np.zeros((5,1))
    yaw_i = np.zeros((5, 1))
    listener(xi, yi,yaw_i)
    cmd_vel_pub=publisher()
    poly_type = 0 # 0 = 默认，1 = 随机生成，2 = 从文件加载, 3 = 随机生成并将结果写入文件
    # NOTE: 随机生成有时候有点bug，但是大部分时候还好。可以指定种子，在grp函数中传参seed=xxx即可
    cur_time_str = datetime.datetime.now().strftime('%Y_%m_%d_%H_%M_%S')
    if poly_type == 0:
        # xrange = 6.5
        # yrange = 4.5
        # crs = np.array([[0, 2 / 5 * yrange],  # 1
        #                 [0, yrange],  # 2
        #                 [1 / 4 * xrange, yrange],  # 3
        #                 [1 / 4 * xrange, 4 / 5 * yrange],  # 4
        #                 [9 / 12 * xrange, 4 / 5 * yrange],  # 5
        #                 [9 / 12 * xrange, yrange],  # 6
        #                 [11 / 12 * xrange, yrange],  # 7
        #                 [11 / 12 * xrange, 9 / 10 * yrange],  # 8
        #                 [xrange, 9 / 10 * yrange],  # 9
        #                 [xrange, 11 / 20 * yrange],  # 10
        #                 [9 / 24 * xrange, 11 / 20 * yrange],  # 11
        #                 [9 / 24 * xrange, 4 / 10 * yrange],  # 12
        #                 [11 / 12 * xrange, 4 / 10 * yrange],  # 13
        #                 [11 / 12 * xrange, 2 / 15 * yrange],  # 14
        #                 [1 / 2 * xrange, 2 / 15 * yrange],  # 15
        #                 [1 / 2 * xrange, 0],  # 16
        #                 [1 / 6 * xrange, 0],  # 17
        #                 [1 / 6 * xrange, 2 / 5 * yrange],  # 18
        #                 [0, 2 / 5 * yrange]])
        # holes=[np.array([[7 / 39 * xrange, 5 / 9 * yrange],[7 / 39 * xrange, 6 / 9 * yrange],[10 / 39 * xrange, 6 / 9 * yrange],[10 / 39 * xrange, 5 / 9 * yrange]])]
        crs = np.array([[0, 1.95],  # 1
                        [0, 4.15],  # 2
                        [0.86, 4.15],  # 3
                        [1.24, 3.97],  # 4
                        [1.24, 3.73],  # 5
                        [4.68, 3.73],  # 6
                        [4.94, 3.43],  # 7
                        [4.94, 2.22],  # 8
                        [2.30, 2.22],  # 9
                        [2.30, 1.26],  # 10
                        [4.47, 1.26],  # 11
                        [4.77, 0.85],  # 12
                        [4.77, 0],  # 13
                        [0.85, 0],  # 14
                        [0.85, 1.95],  # 15
                        [0, 1.95]])
        holes = [np.array([[1.14, 2.62], [1.14, 2.82], [1.3, 2.82],[1.3, 2.62]])]
        xrange=5
        yrange=4.5
        poly_out = Polygon(crs)
        poly_in  = Polygon(holes[0])
        poly = poly_out.difference(poly_in)
        seed_new = 17974358678

        print('Init graph... ', end='')
        start_time = time.perf_counter()
        G, idx2coor = create_graph(step, crs, holes)
        print('{:.2f} s'.format(time.perf_counter() - start_time))
        print('Calculating distance dict... ', end='')
        start_time = time.perf_counter()
        result_dict = dict(nx.shortest_path_length(G, weight='weight'))
        print('{:.2f} s'.format(time.perf_counter() - start_time))
        print('Finish initialization.')




    # x_in = xi.copy()
    # y_in = yi.copy()

    # 创建当前环境的无向图
    # G = init_graph(poly)

    # 创建一个计算的类
    calc = Calc(crs, holes, xi, yi, yaw_i, cmd_vel_pub, mu,ri, result_dict, idx2coor,
                r=r, ke=ke, dt=dt, eps=eps, multi=multi, R=R, k=k, seed=seed_new, xrange=xrange, yrange=yrange)

    # 创建一个画图的类
    plt_vis = PlotVis(calc, savefig=True)

    plt_vis.plt_env()

    # 计算与画图
    plt_vis.cal()
    plt_vis.plt_vis()
    # print(f'other calculation: {(time.time - start_time_part3) / 1e7}\n')

if __name__ == '__main__':
     main()
