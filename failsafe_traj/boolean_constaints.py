#!/usr/bin/env python3
from operators import Y,occ, Yinv
from ref import Ref_path
from intended_arbitary import intended_traj
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from obstacle import obstacles,ego
from invariably_safe_set import *
from initial_state import Initial_state
#from long_prototype import optimize_longitudinal_trajectory
from shapely.geometry import Point, Polygon

x0 = Initial_state()
Tin = intended_traj([(1, 11), (4, 12), (30, 11), (50, 10)])
TTR,T_safe, initial_state_long , initial_state_lat  = x0.initial_state(Tin)  
## define time domain for failsafe trajectory
T = Ref_path([0, 10], [100, 10], 5)
Ob = obstacles(90,20)
Th = T.T # total time horizon  ##ask how to calculate
th = Th 
t = np.linspace(TTR,th,100)

##ego pose from preplanned long motion
#s,v,a,j,u = optimize_longitudinal_trajectory()

def braking_possible():
    s0,v0 = initial_state_long[0:2]
    a_max = 8
    delta_brake = 0.3 
    s_max = Ob.limits()[1]    #obs min long assumed to be preceeding, no correction
    result = True
    for tau in t:
        future_position = s0 + v0 * tau - (0.5) * abs(a_max) * (max(tau - delta_brake, 0))**2 
        if future_position > s_max:
            result = False
            break
    return result

def calculate_evasive_acceleration():
    d0 = initial_state_lat[0]
    d_eva = 3.5
    delta_steer = 0.3
    v_lat = x0.get_v_lat()
    t_GTTC = calculate_GTTC()
    numerator = 2 * d_eva - abs(v_lat) * t_GTTC
    denominator = (t_GTTC - delta_steer) ** 2
    a_eva = numerator / denominator
    return a_eva

def calculate_GTTC():
    s0,v0 = x0[0:2]
    t_values = np.linspace(0, Th - TTR, 100)  # Create a range of time values
    s_predicted = s0 + v0 * t_values  # Calculate the predicted positions

    # Find the time that minimizes the GTTC
    t_GTTC = t_values[np.argmin(s_predicted - s_max)]
    return t_GTTC

##invariable safe set constraint

if __name__ == "__main__":
    print(initial_state_lat[0])
    print(lateral_feasible())



