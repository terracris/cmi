import numpy as np
from math import cos, sin, pi

# angles must be in radians
theta_1 = pi/4
theta_2 = pi/4
theta_3 = pi/4

# link lengths are in mm (I wonder if this is okay)
# TODO measure link lenghts (cannot really use solidworks for this)
L1 = 126.40
L2 = 46
L3 = 150
L4 = 15
L5 = 194.10

num_joints = 3
y_offset = -6.5  # y offset of the end effector relative to the base. 

# cos and sin are in radians

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

# account for an offset
base_ee_trans[1][3] += y_offset

print(np.round(base_ee_trans, decimals=4))
