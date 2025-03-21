import numpy as np
from convex import convex
def find_reflex(D,Cd,crs,holes,x_in,y_in,R):
    # ---------------------------------------------------------------------------#
    # #FIND_REFLEX 找到所有节点的非凸点
    # #输出非凸点坐标及索引
    # ---------------------------------------------------------------------------#
    Pr=np.zeros((0, 2))
    Cr=[]
    c1=convex(crs)  #判断环境中的凸点和非凸
    c2=np.where (c1==-1)[:][0]  #找到环境中所有非凸点
    for ij in range (len(x_in)):  #找每个节点对应的非凸点
        pr=np.zeros((0, 2))          #存储当前节点的非凸点坐标
        for i in range(len(c2)):  #如果非凸点在节点的维诺边界且与节点距离小于R则会挡住节点视线
            Di=D[Cd[ij],:]
            diffs=Di-crs[c2[i],:]
            norms=np.linalg.norm(diffs, axis=1)
            c3=np.where(norms<2e-3)
            if len(c3)!=0:
                d=np.linalg.norm([x_in[ij]-crs[c2[i],0] , y_in[ij]-crs[c2[i],1]])
                if d<R:
                    pr=np.concatenate((pr,crs[c2[i],:][np.newaxis,:]),axis=0)
        for j in holes:
            for k in range(j.shape[0]):
                Di = D[Cd[ij], :]
                diffs = Di - j[k, :]
                norms = np.linalg.norm(diffs, axis=1)
                c3 = np.where(norms < 2e-3)
                if len(c3) != 0:
                    d = np.linalg.norm([x_in[ij] -  j[k, 0], y_in[ij] -  j[k, 1]])
                    if d < R:
                        pr = np.concatenate((pr,  j[k, :][np.newaxis, :]), axis=0)

        Cr.append(np.arange(Pr.shape[0], Pr.shape[0] + pr.shape[0]))
        Pr=np.concatenate((Pr, pr))

    return Pr,Cr