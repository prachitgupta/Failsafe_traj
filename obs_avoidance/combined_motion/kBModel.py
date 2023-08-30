#!/usr/bin/env python3

import numpy as np
import matplotlib.pyplot as plt

# Parameters
start = np.array([0, 10, 0])  # [x, y, theta]
goal = np.array([40, 10])
T = 5  # Time horizon
dt = 0.1  # Time step

# Define the kinematic bicycle model
def kinematic_bicycle_model(state, control):
    x, y, theta = state
    v, omega = control
    x_next = x + v * np.cos(theta) * dt
    y_next = y + v * np.sin(theta) * dt
    theta_next = theta + omega * dt
    return np.array([x_next, y_next, theta_next])

# Generate smooth control inputs (constant velocity and zero angular velocity)
num_steps = 100
v = np.linspace(0, np.linalg.norm(goal - start[:2]) / T, num_steps-1)
omega = np.zeros(num_steps-1) #NO ROTATION
controls = np.vstack((v, omega))

# Simulate the trajectory
trajectory = [start]
current_state = start
print(current_state)
for control in controls.T:
    current_state = kinematic_bicycle_model(current_state, control)
    trajectory.append(current_state)
trajectory = np.array(trajectory)

# Plot the trajectory
plt.figure(figsize=(8, 6))
print(trajectory[:, 0].shape)
plt.plot(trajectory[:, 0], trajectory[:, 1], marker='o')
plt.plot(goal[0], goal[1], marker='x', color='red')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Intended Trajectory')
plt.grid(True)
plt.show()
