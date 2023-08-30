#!/usr/bin/env python3
import cvxpy as cp
from operators import Y,occ
from ref import Ref_path
from intended_arbitary import intended_traj
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.ticker import MultipleLocator
from obstacle import obstacles,ego
from invariably_safe_set import *
from initial_state import Initial_state


X0 = Initial_state()
Tin = intended_traj([(0, 10, 0),(50, 10)])
x0 = X0.initial_state(Tin)[2]  
TTR = X0.initial_state(Tin)[0]  
Ob = obstacles(20,10)
## define time domain for failsafe trajectory
T = Ref_path([0, 10], [100, 10], 5)
Th = T.T # total time horizon  ##ask how to calculate
num_steps = 100
t = np.linspace(TTR,Th,100)
dt = t[1] - t[0]
# Initial and final conditions
s0, v0, a0, j0, snap0 = x0  # Initial state (position, velocity, acceleration, jerk))
w_a = 1.0  # Weight for acceleration term in the cost function
w_j = 1.0  # Weight for jerk term in the cost function
# Additional deceleration limits and slack weights
a_lim1 = -6.0  # Lower deceleration limit
a_lim2 = -5.0  # Upper deceleration limit
w_ς1 = 1.0  # Weight for slack variable ς_lon,1
w_ς2 = 2.0  # Weight for slack variable ς_lon,2
status = True
a1,b,c = 0,0,0
#v_intervals = np.linspace(vmin, vmax,  + 1)

def optimize_longitudinal_trajectory(i,f):
    global status
    # Initialize the control input variable (acceleration, 4th derivative of s)
    u_lon = cp.Variable(num_steps) #generated tuple of 51 control inputs

    # Define the state variables (position, velocity, acceleration, jerk)
    s = cp.Variable(num_steps)
    v = cp.Variable(num_steps)
    a = cp.Variable(num_steps)
    j = cp.Variable(num_steps)
    snap = cp.Variable(num_steps)
     # Define the slack variables (slack for deceleration limits)
    ς_lon1 = cp.Variable(num_steps, nonneg=True)
    ς_lon2 = cp.Variable(num_steps, nonneg=True)

    # Define the cost function to be minimized (minimize jerk)
    cost = cp.sum_squares(a) * w_a + cp.sum_squares(j) * w_j + cp.sum(ς_lon1) * w_ς1 + cp.sum(ς_lon2) * w_ς2
    s_max = Ob.limits()[0] if  Ob.limits()[0] > s0 else 95  ##only for preceeding in lane
    s_min = Ob.limits()[1] if  Ob.limits()[1] < s0 else 0
    vmin = 0
    vmax = 40
    amax = 8
    amin = -10
    jmax = 8
    jmin = -6
    delta_brake = 0.3
    ##doubt a_d_max =  constant constraint or has to be calculated
    a_d_max = 8
    a_s_max_b = 0.6
    a_s_max = 8
    v_b = 0
    h = 5  # Number of linear segments
    # global a1,b,c
    # # Linear functions for safe distance approximation
    # def g_i(x, x_i, x_next):
    #     global a1,b,c
    #     f_x_i = a1 * x_i ** 2 + b * x_i + c
    #     f_x_next = a1* x_next ** 2 + b * x_next + c
    #     slope = (f_x_next - f_x_i) / (x_next - x_i)
    #     print((x - x_i))
    #     return slope * (x - x_i) + f_x_i

    # # Piecewise linear approximation of safe distance
    # def safe_distance(v):
    #     global v_intervals
    #     v_intervals = np.linspace(vmin, vmax, h + 1)
    #     safe_distances = [g_i(v, v_intervals[i], v_intervals[i + 1]) for i in range(h)]
    #     #print(cp.vstack(safe_distances))
    #     return cp.vstack(safe_distances)

    # # Longitudinal position constraint incorporating safe distance
    # def longitudinal_position_constraint(v, s_max):
    #     return v + safe_distance(v) - s_max

    # Constraints: initial conditions, and dynamics constraint
    constraints = [
        s[0] == i[0], v[0] == i[1], a[0] == i[2], j[0] == i[3], 
    ]

    #final position
    constraints += [
        s[99] == f[0], 
    ]
    
    ## LTI Systems
    for k in range(num_steps - 1):
        delta_eva = calculate_evasive_distance(v0, a_d_max, 3.5, 0.3, 0, 0.6)
        delta_eva = 3
        '''
        if (a_s_max_b <= a_s_max or False):
            a1 = 1 / (2 * (abs(a_s_max_b) - abs(a_s_max)))
            b = (-2*(v_b - abs(a_s_max_b)*delta_brake))/(2 * (abs(a_s_max_b) - abs(a_s_max))) + delta_brake
            c = ((v_b - abs(a_s_max_b)*delta_brake)**2)/(2 * (abs(a_s_max_b) - abs(a_s_max))) -v_b*delta_brake + 1/2*abs(a_s_max_b)*delta_brake**2
    
        ##assumed delta_safe 2 only for simplicity
        a1 = 1 / (2 * abs(a_s_max))
        b = delta_brake
        c = (-v_b**2 / (2 * abs(a_s_max_b)))
        v_
        safe_distance_constraint = longitudinal_position_constraint(v_ego, s_max)
        
        ##can't deal with max so break in linear
       
        for i in range(h):
             constraints += [
               safe_distance_constraint[i] >= 0
             ]
     '''
        constraints += [
            ##state space 
            s[k+1] == s[k] + dt*v[k] + 0.5*(dt**2)*a[k] + (1/6)*(dt**3)*j[k] + (1/24)*(dt**4)* u_lon[k],
            v[k + 1] == v[k] + dt * a[k] + 0.5 * (dt ** 2) * j[k] + (1/6) * (dt ** 3) *  u_lon[k],
            a[k + 1] == a[k] + dt * j[k] + (1/2) * (dt ** 2) *  u_lon[k],
            j[k + 1] == j[k] + dt * u_lon[k],
            # # Fourth derivative of s is equal to u_lon (control input)

            ##actual lti discrete approx
            # s[k+1] == s[k] + dt*v[k] ,
            # v[k + 1] == v[k] + dt * a[k] ,
            # a[k + 1] == a[k] + dt * j[k] ,
            # j[k + 1] == j[k] + dt * u_lon[k],

            a[k] >= a_lim1 - ς_lon1[k],  # Slack variable for deceleration limit 1
            a[k] >= a_lim2 - ς_lon2[k],  # Slack variable for deceleration limit 2

            ## obstacle avoidance constraint
            # s[k] >= s_min,
            # s[k] <= s_max,
            # Velocity inequality constraints
            v[k] >= vmin,
            v[k] <= vmax,
            # Acceleration inequality constraints
            a[k] >= amin,
            a[k] <= amax,
            # # jerk inequality constraints
            # j[k] >= jmin,
            # j[k] <= jmax,

            ##invariably safe set constraints
            # s[k] + delta_eva <= s_max ,

            ## evasive acc constraints
        ]    
    # Create the optimization problem
    problem = cp.Problem(cp.Minimize(cost), constraints)

    # Solve the problem using the chosen solver (e.g., ECOS)
    problem.solve(solver=cp.ECOS)
    status = problem.status
    if(status == "optimal"):
        return s.value, v.value, a.value, j.value, u_lon.value
    else:
        print("infeasible")
        return (status)

if __name__ == "__main__":
    # Generate the longitudinal trajectory
    s_traj, v_traj, a_traj, j_traj, u_lon_traj = optimize_longitudinal_trajectory()
    print(status)
    # Time vector
    if(status == "optimal"):
    # Plotting trajectories
        print("haa bhai mai hi hu")
        plt.figure(figsize=(10, 6))

        plt.subplot(3, 1, 1)
        plt.plot(t, s_traj, label='Position (s)')
        plt.xlabel('Time (s)')
        plt.ylabel('Position (s)')
        plt.legend()

        plt.subplot(3, 1, 2)
        plt.plot(t, v_traj, label='Velocity (v)')
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (v)')
        plt.legend()

        plt.subplot(3, 1, 3)
        plt.plot(t, u_lon_traj, label='Control Input (u_lon)')
        plt.xlabel('Time (s)')
        plt.ylabel('Control Input (u_lon)')
        plt.legend()

        plt.tight_layout()
        plt.show()
    else:
        print("no feasible sol")
