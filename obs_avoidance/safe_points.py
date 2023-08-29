#!/usr/bin/env python3
import cvxpy as cp
from operators import Y,occ
import numpy as np
from obstacle import obstacles,ego
from intended_arbitary import intended_traj

Ob = obstacles(40,10)


def get_safe_point_lat(s,d):
    d_max = Ob.limits()[2]
    Epsilon = 0.5
    d_min_safe = d_max +  Epsilon
    safe_y =  d_min_safe + 1  ##radius of occ defined as 1
    safe_x = s
    return safe_x , safe_y

def get_safe_point_long(s,d):
    s_max = Ob.limits()[0]
    Epsilon = 0.5
    s_min_safe = s_max +  Epsilon
    safe_x = s_min_safe + 1  ##radius of occ defined as 1
    safe_y = d
    return safe_x , safe_y

def get_safe_point_traj(s,d,Tin):
    st,dt = Tin.generate_curvilinear_path()[:,0], Tin.generate_curvilinear_path()[:,1]
    points = np.column_stack((st,dt))

    # Find the nearest point on the reference trajectory to the given (x, y) coordinates
    distances = np.sqrt((st[:] - s) ** 2 + (dt[:] - d) ** 2)
    nearest_index = np.argmin(distances)

    sr = st[nearest_index]
    dr = dt[nearest_index]
    print(dr)

    return sr, dr