import numpy as np
import matplotlib.pyplot as plt

# Define linear approximation parameters
v_min = 0.0  # Minimum velocity
v_max = 10.0  # Maximum velocity
h = 5  # Number of linear segments

# Linear functions for safe distance approximation
def g_i(x, v_i, v_next):
    return (x - v_i) * (1 / (v_next - v_i))

# Piecewise linear approximation of safe distance
def safe_distance(x_lon, v_lon):
    v_intervals = np.linspace(v_min, v_max, h + 1)
    safe_distances = [g_i(x_lon, v_intervals[i], v_intervals[i + 1]) for i in range(h)]
    return max(safe_distances)

# Longitudinal position constraint incorporating safe distance
def longitudinal_position_constraint(x_lon, v_lon, s_max):
    return x_lon + safe_distance(x_lon, v_lon) - s_max

# Example usage
x_lon = 5.0  # Current longitudinal position
v_lon = 6.0  # Current longitudinal velocity
s_max = 100.0  # Maximum allowed longitudinal position

# Evaluate safe distance and constraint
safe_dist = safe_distance(x_lon, v_lon)
constraint_value = longitudinal_position_constraint(x_lon, v_lon, s_max)

print("Safe Distance:", safe_dist)
print("Longitudinal Position Constraint:", constraint_value)
