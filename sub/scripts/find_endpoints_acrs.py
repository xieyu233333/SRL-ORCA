import numpy as np
import math
def find_endpoints_acrs(Dci,R):
    start_point=[]
    end_point=[]
    start_point=start_point+[0]
    for i in range(1,len(Dci)-1):
        d1=Dci[i,:]-Dci[i-1,:]
        d2=Dci[i,:]-Dci[i+1,:]
        if np.linalg.norm(d1)>R*2*math.pi*5/360:
            end_point=end_point+[i]
            start_point=start_point+ [i-1]
        elif np.linalg.norm(d2)>R*2*math.pi*5/360:
            start_point=start_point+ [i]
            end_point=end_point+[i+1]
    end_point=end_point+[len(Dci)-1]
    return start_point,end_point