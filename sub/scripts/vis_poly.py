import numpy as np
from inpolygon import inpolygon
from get_visible_r import get_visible_r


def vis_poly(crs, holes, x_in,y_in,R,poly):
    #VIS_POLY 此处显示有关此函数的摘要
    #   此处显示详细说明
    Vpoly=np.zeros((0, 2))
    Cpoly=[]
    w_i = np.arange(0, 2 * np.pi, 2 * np.pi / 720)[0::4]
    for p in range(len(x_in)):
        lemda=0.03            #半径变化的步长
        boundsetxi=np.empty(0)        #记录第i个节点的维诺边界
        boundsetyi=np.empty(0)
        xn = x_in[p]                               #存储i节点的坐标（数）
        yn = y_in[p]
        r = R * 2
        r_test = np.arange(0,r+lemda,lemda)
        if inpolygon(xn,yn,crs[:,0],crs[:,1], holes):
            r_i_T = np.transpose([r_test])
            pts_x = xn + r_i_T @ np.cos(w_i)[np.newaxis, :]
            pts_y = yn + r_i_T @ np.sin(w_i)[np.newaxis, :]
            result_poly = 1 - inpolygon(pts_x, pts_y, crs[:, 0], crs[:, 1], holes)[0]
            val = np.max(result_poly, axis=0)
            idx = np.argmax(result_poly, axis=0)

            # 6.
            max_idx = len(r_test) - 1
            idx = idx * val + max_idx * (1 - val)
            boundsetxi = pts_x[idx, np.arange(idx.shape[0])]
            boundsetyi = pts_y[idx, np.arange(idx.shape[0])]
        boundsetxi=np.transpose(boundsetxi)
        boundsetyi=np.transpose(boundsetyi)
        boundseti=np.concatenate((boundsetxi[:, np.newaxis],boundsetyi[:, np.newaxis]),axis=1)
        # if len(boundseti)!=0:
        #     boundseti = boundseti[0::4]
        Cpoly.append(np.arange(Vpoly.shape[0], Vpoly.shape[0] + boundseti.shape[0]))
        Vpoly=np.concatenate((Vpoly, boundseti))
    return Vpoly,Cpoly
