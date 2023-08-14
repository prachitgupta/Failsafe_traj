#!/usr/bin/env python3
from operators import Y,occ
from ref import Ref_path
from intended_arbitary import intended_traj
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from obstacle import obstacles,ego
from invariably_safe_set import *
from initial_state import Initial_state
from boolean_constraints import *
from lateral_constraints import *

Tin = intended_traj([(1, 11), (4, 12), (30, 11), (50, 10)])
x0 = Initial_state()
status = x0.initial_state(Tin)

if(status == "verified"):
    print("execute intended trajectory")

elif(status == "boom"):
    print("don't bother it collided")

else:
    TTR,T_safe, initial_state_long , initial_state_lat = status
    ## main algorithm

    #obtain long constraints    

    ##plan long trajectory irrespective of whether you will be executing it or not
    ##long state(t) used for lateral
    if(braking_possible()):
        can_stop = True
        planned_long = plan_long("without") ##without evasive constraint
    else:
        calculate_evasive_acceleration()
        planned_long = plan_long("with")  ##with
    
    ##assumed only shift to right lane, set based reachability
    
    ##doubtful part
    #maybe real time trajectories are continousily generated and getting verified
    #but we assumed a pre planned fixed trajectory so no previousily verified demonstrated 
    #at present if lateral infeasible and can stop, stop if both fails just print execute previous

    if(not lateral_feasible()):
        if(can_stop):
            T_verified = append(T_safe,planned_long)
            execute()
        else:
            print("no new failsafe trajectory found, execute previous verified")   
    else:
        planned_lateral = plan_lat()
        T_verified = append(T_safe,planned_lat)
        execute()

        ##genrated first verified trajectory

