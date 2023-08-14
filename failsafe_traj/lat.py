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

## occupancy assumed as a circle not 3 equidistant circles as explained in paper
X0 = Initial_state()
Tin = intended_traj([(1, 11), (4, 12), (30, 11), (50, 10)])
x0 = X0.initial_state(Tin)[3]  
v =  X0.initial_state(Tin)[2][1]
TTR = X0.initial_state(Tin)[0]  
Ob = obstacles(40,10)
## define time domain for failsafe trajectory
T = Ref_path([0, 10], [100, 10], 5)
Th = T.T # total time horizon  ##ask how to calculate
alpha = T.orientation() ##orientation assumed to be constant of ref so ignored time horizon
t = np.linspace(TTR,Th,100)
dt = t[1] - t[0]
# Initial and final conditions
d0,theta0, K0, Kdot0, Kddot0 = x0  # Initial state (position, velocity, acceleration, jerk))
num_steps= 100

# Define lateral motion parameters based on the longitudinal trajectory

# State-space representation

# Main function for lateral motion optimization
def optimize_lateral_motion():
    # Initialize state vector and time vector
    x_lat = cp.Variable((4, num_steps))
    time = t

    # Control input (u_lat, second derivative of curvature)
    u_lat = cp.Variable((1, num_steps))  # Modify this to your desired control input

    # Define the cost function to be minimized (add your desired cost function here)
    # For example, to minimize the total control effort (sum of squared control inputs):
    w_d = 1.0
    w_theta = 1.0
    w_kappa = 1.0
    w_kappa_dot = 1.0

    # Cost function
    cost = (
        w_d * cp.sum_squares(x_lat[0, :]) +
        w_theta * cp.sum_squares(x_lat[1, :] - alpha[]) +
        w_kappa * cp.sum_squares(x_lat[2, :]) +
        w_kappa_dot * cp.sum_squares(x_lat[3, :])
    )

    # Constraints: initial conditions, and dynamics constraint
    constraints = [
        x_lat[:, 0] == np.array([d0, theta0, K0, Kdot0]).reshape(-1, 1),  # Initial state
    ]
    
    # State space constraints
    for k in range(num_steps - 1):
        z_lat = x_lat[1, k] - alpha  # Orientation difference
        
        A_lat = np.array([[0, v[k], 0, 0],
                          [0, 0, v[k], 0],
                          [0, 0, 0, 1],
                          [0, 0, 0, 0]])
        
        B_lat = np.array([[0],
                          [0],
                          [0],
                          [1]])
        
        C_lat = np.array([[-v[k]],
                          [0],
                          [0],
                          [1]])
        
        u_lat_k = np.array([u_lat[:, k]])
        
        # Ensure that u_lat represents the second derivative of curvature (derivative of x_lat[3,:])
        x_next = x_lat[:, k] + dt * (A_lat @ x_lat[:, k] + B_lat @ u_lat_k + C_lat @ z_lat)
        u_lat_k[0] = (x_next[3] - x_lat[3, k]) / dt
        
        constraints += [
            x_lat[:, k + 1] == x_next,
        ]

    # Create the optimization problem
    problem = cp.Problem(cp.Minimize(cost), constraints)

    # Solve the problem using the chosen solver (e.g., ECOS)
    problem.solve(solver=cp.ECOS)

    return x_lat.value, u_lat.value, time


if __name__ == "__main__":
    # Optimize lateral motion
    x_lat_traj, u_lat_traj, time = optimize_lateral_motion()

    # Extract trajectories from optimization results
    y_traj = x_lat_traj[0, :].flatten()
    y_dot_traj = x_lat_traj[1, :].flatten()
    psi_traj = x_lat_traj[2, :].flatten()
    r_traj = x_lat_traj[3, :].flatten()

    # Plotting trajectories
    plt.figure(figsize=(10, 6))

    plt.subplot(4, 1, 1)
    plt.plot(time, y_traj, label='Lateral Position (y)')
    plt.xlabel('Time (s)')
    plt.ylabel('Lateral Position (y)')
    plt.legend()

    plt.subplot(4, 1, 2)
    plt.plot(time, y_dot_traj, label='Lateral Velocity (y_dot)')
    plt.xlabel('Time (s)')
    plt.ylabel('Lateral Velocity (y_dot)')
    plt.legend()

    plt.subplot(4, 1, 3)
    plt.plot(time, psi_traj, label='Yaw Angle (ψ)')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Angle (ψ)')
    plt.legend()

    plt.subplot(4, 1, 4)
    plt.plot(time, r_traj, label='Yaw Rate (r)')
    plt.xlabel('Time (s)')
    plt.ylabel('Yaw Rate (r)')
    plt.legend()

    plt.tight_layout()
    plt.show()
