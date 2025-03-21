import numpy as np
import math
from shapely.geometry import Polygon
def find_neighbors(x_in,y_in,R,D,Cd):
# ---------------------------------------------------------------------------#
# 寻找每个机器人对应的邻居节点
# ---------------------------------------------------------------------------#
    Vn=np.zeros((0, 1))
    Cn=[]
    for ij in range(len(x_in)):
        xs = np.concatenate((x_in[:ij], x_in[ij + 1:]), axis=0)
        ys = np.concatenate((y_in[:ij], y_in[ij + 1:]), axis=0)
        xn = x_in[ij]                               #存储i节点的坐标（数）
        yn = y_in[ij]
        vn=np.zeros((0, 1))
        if Cd[ij] is None:
            Dpoly = None
        else:
            Dpoly = Polygon(D[Cd[ij], :])
        for i in range(len(xs)):
            diffs=np.concatenate((x_in-xs[i],y_in-ys[i]),axis=1)
            norms=np.linalg.norm(diffs, axis=1)
            n=np.array(np.where(norms < 0.00001))                 ##找到当前节点在原数组的位置
            n=n[0][0]
            dd = math.sqrt((xs[i] - xn) ** 2 + (ys[i] - yn) ** 2)
            if Cd[n] is None:
                Npoly = None
            else:
                Npoly = Polygon(D[Cd[n], :])
            if dd <=2*R and Dpoly.overlaps(Npoly) and n!=ij:    ##在通信范围内且两个单元有交集可以作为S的j
                vn=np.append(vn,n)
        Cn.append(np.arange(Vn.shape[0], Vn.shape[0] + vn.shape[0]))
        Vn=np.append(Vn,vn)

    Vn=Vn.astype(int)
    return Vn,Cn