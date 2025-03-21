import math
from shapely.geometry import Point, Polygon, LineString

def get_visible_r(poly, xn, yn, theta_d, r_test):
    idx_min = 0
    idx_max = len(r_test)-1
    idx_current = 1
    while idx_max > idx_min:
        radian = math.radians(theta_d)
        xa = (xn+r_test[idx_current]*math.cos(radian))
        ya = (yn+(r_test[idx_current])*math.sin(radian))
        point1 = Point(xa, ya)
        point2 = Point(xn, yn)
        # 定义两点之间的线段
        lineseg=LineString([point1, point2])
        if   lineseg.within(poly):
            idx_min = idx_current
        else:
            idx_max = idx_current - 1
        idx_current = math.floor((idx_max + idx_min) / 2)
        if idx_current == idx_min and idx_max > idx_min:
            idx_current = idx_current + 1
        elif idx_current == idx_max and idx_max > idx_min:
            idx_current = idx_current - 1
    xa = (xn+(r_test[idx_current])*math.cos(radian))
    ya = (yn+(r_test[idx_current])*math.sin(radian))
    return xa,ya