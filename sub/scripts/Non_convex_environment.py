import numpy as np
from KOrderVoronoi import KOrderVoronoi
from poly2cw import poly2cw
from shapely.geometry import Polygon
def Non_convex_environment(crs, x_in, y_in, R, k,poly):
    # ---------------------------------------------------------------------------#
    # 非凸环境的k阶维诺图
    # ---------------------------------------------------------------------------#
    rgx = max(crs[:,0])-min(crs[:,0])
    rgy = max(crs[:,1])-min(crs[:,1])
    rg = max(rgx, rgy)
    midx = (max(crs[:, 0])+min(crs[:, 0])) / 2
    midy = (max(crs[:, 1])+min(crs[:, 1])) / 2

    ##add 4 additional edges
    xA = np.concatenate((x_in, midx + np.array([0, 0, -5*rg, 5*rg])), axis=0)
    yA = np.concatenate((y_in, midy + np.array([-5*rg, 5*rg, 0, 0])), axis=0)

    vi, ci = KOrderVoronoi(xA, yA, R, k, crs) #计算维诺图，返回维诺单元的顶点和索引
    #remove the last 4 cells
    C = ci[: - 4]
    V = vi
    #use Polybool to crop the cells
    #Polybool for restriction of polygons to domain.
    for ij in range(len(C)) :
        # X2, Y2 = poly2cw(V[C[ij], :]) #对顶点顺时针排序
        # X2 = V[C[ij], [0]]
        # Y2 = V[C[ij], [1]]
        poly_ij = Polygon(V[C[ij]])
        inter_poly = poly.intersection(poly_ij) #求环境和维诺单元的相交部分
        xb,yb=inter_poly.exterior.xy
        ix=np.full((1, len(xb)), np.nan)
        for il in range (len(xb)):
            if np.any(V[: ,0] == xb[il]) and np.any(V[:, 1] == yb[il]): #一旦维诺单元边界和环境有相交
                ix1 = np.where(V[:, 0] == xb[il])[0]
                ix2 = np.where(V[:, 2] == yb[il])[0]
                for ib in range(len(ix1)):
                    if np.any(ix1[ib] == ix2):
                        ix[il] = ix1[ib]
                if np.isnan(ix[il]):
                    lv = len(V)
                    V = np.concatenate((V, np.array([[xb[il], yb[il]]])), axis=0)
                    ix[il] = lv + 1
            else:
                lv = len(V)
                V = np.concatenate((V, np.array([[xb[il], yb[il]]])), axis=0)
                ix[il] = lv + 1
        C[ij] = ix
    return C,V