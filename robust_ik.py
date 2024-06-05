import numpy as np
from math import cos, sin, pi, degrees
from scipy.optimize import minimize
import time

# Forward kinematics function
def forward_kinematics(thetas, L1, L2, L3, L4, L5, y_offset):
    theta_1, theta_2, theta_3 = thetas
    A_1 = np.array([[cos(theta_1), 0, -sin(theta_1), (L2*cos(theta_1))],
                    [sin(theta_1), 0, cos(theta_1),  (L2*sin(theta_1))],
                    [0,           -1,  0,                           L1],
                    [0,            0 , 0,                            1]
                    ])

    A_2 = np.array([[cos(theta_2 - (pi/2)), -sin(theta_2 - (pi/2)),  0, ((L3+L4)*cos(theta_2 - (pi/2)))],
                    [sin(theta_2 - (pi/2)), cos(theta_2 - (pi/2)),   0,  ((L3+L4)*sin(theta_2 - (pi/2)))],
                    [0,                     0,                       1,                            0],
                    [0,                     0 ,                      0,                            1]
                    ])

    A_3 = np.array([[cos(theta_3 + (pi/2)), -sin(theta_3 + (pi/2)),  0, ((L5)*cos(theta_3 + (pi/2)))],
                    [sin(theta_3 + (pi/2)), cos(theta_3 + (pi/2)),   0,  ((L5)*sin(theta_3 + (pi/2)))],
                    [0,                     0,                       1,                            0],
                    [0,                     0 ,                      0,                            1]
                    ])

    base_ee_trans = np.eye(4)
    transformations = [A_1, A_2, A_3]
    
    for intermediate_trans in transformations:
        base_ee_trans = np.dot(base_ee_trans, intermediate_trans)
    
    base_ee_trans[1][3] += y_offset

    return base_ee_trans

# Objective function
def objective(thetas, target_position, L1, L2, L3, L4, L5, y_offset):
    fk_result = forward_kinematics(thetas, L1, L2, L3, L4, L5, y_offset)
    ee_position = fk_result[:3, 3]  # Extract the end-effector position
    error = np.linalg.norm(ee_position - target_position)  # Compute the Euclidean distance
    return error

# Inverse kinematics solver using L-BFGS-B
def inverse_kinematics(target_position, initial_guess, L1, L2, L3, L4, L5, y_offset, bounds, max_iter=1000, tol=1e-6):
    options = {
        'maxiter': max_iter,
        'disp': False,  # Display convergence messages
    }
    start = time.time()    
    result = minimize(objective, initial_guess, args=(target_position, L1, L2, L3, L4, L5, y_offset), method='L-BFGS-B', bounds=bounds, options=options, tol=tol)
    end = time.time()

    elapsed_time = end - start
    return result.x, result, elapsed_time

# Parameters
L1 = 126.40
L2 = 46
L3 = 150
L4 = 15
L5 = 194.10
y_offset = -3.5

# Target end-effector position (example)
target_position = np.array([200, 100, 150])

# Initial guess for the joint angles
initial_guess = np.array([pi/4, pi/4, pi/4])

# Bounds for each joint angle
bounds = [(-pi/2, pi/3), (-pi/18, 2*pi/3), (-4*pi/9, 4*pi/9)]

# Calculate inverse kinematics using L-BFGS-B
optimized_thetas, result, elapsed_time = inverse_kinematics(target_position, initial_guess, L1, L2, L3, L4, L5, y_offset, bounds)

print("Optimized joint angles (degrees):", [round(degrees(x), 4) for x in optimized_thetas])
print("Optimization success:", result.success)
print("Message:", result.message)
print(f"Elapsed time: {elapsed_time:.4f} seconds")
