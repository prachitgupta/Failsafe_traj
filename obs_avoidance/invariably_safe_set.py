#!/usr/bin/env python3
from operators import Y,occ
from ref import Ref_path
from intended_arbitary import intended_traj
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from obstacle import obstacles,ego



###  collision free states  F
def is_in_F(ego,obs):
    in_F = True
    obstacle_max_long, obstacle_min_long,obstacle_max_lat,obstacle_min_lat = obs.limits()
    ego_max_long, ego_min_long,ego_max_lat ,ego_min_lat = ego.limits()
    if ego_max_long > obstacle_min_long and ego_min_long < obstacle_max_long:
        # Check for intersection in the lateral direction
        if ego_max_lat > obstacle_min_lat and ego_min_lat < obstacle_max_lat:
            in_F = False

    return  in_F

### invariable safe set using online verification technique

##safe distance 

def calculate_safe_distance(v_ego, a_s_max, v_b, a_s_max_b, delta_brake):
    if a_s_max_b < a_s_max and v_b < v_ego and (v_ego / a_s_max) < (v_b / a_s_max_b):
        v_b_star = v_b - abs(a_s_max_b) * delta_brake if delta_brake <= v_b / abs(a_s_max_b) else 0
        delta_safe_1 = ((v_b_star - abs(a_s_max_b) * delta_brake - v_ego) ** 2) / (
                        -2 * (abs(a_s_max_b) - abs(a_s_max)) +
                        1 / 2 * abs(a_s_max_b) * delta_brake**2 - v_b * delta_brake +
                        v_ego * delta_brake)
        return delta_safe_1

    else:
        delta_safe_2 = (-v_b**2/(2*(abs(a_s_max_b)))) - (v_ego**2)/(-2*abs(a_s_max)) + (v_ego*delta_brake)
        #print(f"safe _dis =  {delta_safe_2}")
        return delta_safe_2


##evasive distanc2
def calculate_evasive_distance(v_ego, a_d_max, d_eva, delta_steer, v_b, a_s_max_b):
    t_eva = (2 * d_eva / a_d_max) + delta_steer
    t_b = min(t_eva, v_b / abs(a_s_max_b))
    delta_s_b = v_b * t_b - 0.5 * abs(a_s_max_b) * (t_b ** 2)
    delta_eva = v_ego * t_eva - delta_s_b

    return delta_eva

##S(t) = S1(t) U S2(t)
#constraint for S1(t)
def is_in_S1(ego,obs):
    obstacle_max_long, obstacle_min_long,obstacle_max_lat,obstacle_min_lat = obs.limits()
    ego_max_long, ego_min_long,ego_max_lat ,ego_min_lat = ego.limits()
    in_S1 = True
    v_ego = ego.v_ego
    delta_safe = calculate_safe_distance(v_ego, 2, 0, 0.6, 0.3) ##data from experiments performed
    if(ego_max_long > (obstacle_min_long - delta_safe)):
        in_S1 = False
    return in_S1

#constraint for S2(t)
def is_in_S2(ego,obs):
    obstacle_max_long, obstacle_min_long,obstacle_max_lat,obstacle_min_lat = obs.limits()
    ego_max_long, ego_min_long,ego_max_lat ,ego_min_lat = ego.limits()
    in_S2 = True
    v_ego = ego.v_ego
    delta_eva = calculate_evasive_distance(v_ego, 8.0, 3.5, 0.3, 0, 0.6) ##data from experiments performed
    if(ego_max_long > (obstacle_min_long - delta_eva)):
        in_S2 = False
    return in_S2

def is_in_S(ego,obs):
    if(is_in_S1(ego,obs) or is_in_S2(ego,obs)):
        return True
    else:
        return False

'''
Ob = obstacles(20,4)
ego = ego(1,5,0)

if(not is_in_F(ego,Ob)):
    print("BOOM!!")
else:
    if (is_in_S(ego,Ob)):
        print("yesss")
    else:
        print("TTR")
'''

