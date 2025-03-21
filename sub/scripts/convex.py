import numpy as np
def convex(X):
    # Array c is 1 where polygon is convex, -1 where curve is concave
    #  Input: X = n by 2 array of x and y values
    # Are Mjaavatten, November 2021
    n=X.shape[0]
    Z=np.concatenate((X,np.zeros((n,1))),axis=1)
    c=np.full((n,1), np.nan)
    for i in range(1,n-1):
        cc = np.cross(Z[i,:]-Z[i-1,:],Z[i+1,:]-Z[i-1,:])
        c[i] = -np.sign(cc[2])
    # Change sign if the points are not in clockwise order:
    clockwise = np.sign(np.sum(np.roll(X[:,0],-1)*X[:,1]-X[:,0]*np.roll(X[:,1],-1)))
    c = c*clockwise
    return c