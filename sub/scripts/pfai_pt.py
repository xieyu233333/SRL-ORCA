import numpy as np


def pfai_pt(x, y, mu, r, ke, fai):
    q = np.concatenate((x[:,np.newaxis], y[:,np.newaxis]), axis=1)
    dmu_dt = -ke * (mu - r)
    dmu_dt = dmu_dt.T
    fai_pt = (q - mu) @ dmu_dt * fai
    return fai_pt
