pulses_per_rev = 200
enable_pin = 37

# joint 1
pulse_pin_j1 = 11
dir_pin_j1 = 13
homing_pin_j1 = 7
gear_ratio_j1 = 4
home_count_j1 = -140
max_speed_j1 = 75
max_positive_angle_j1 = 60
max_negative_angle_j1 = -90
homing_direction_j1 = Stepper.CCW

# joint 2
pulse_pin_j2 = 19
dir_pin_j2 = 21
homing_pin_j2 = 23
gear_ratio_j2 = 5 * 5.18
home_count_j2 = -145
max_speed_j2 = 75
# gonna need to update kinematics to account for the joint limits:
# like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
max_positive_angle_j2 = 115
max_negative_angle_j2 = -10
homing_direction_j2 = Stepper.CCW

# joint 3
pulse_pin_j3 = 29
dir_pin_j3 = 31
homing_pin_j3 = 33
gear_ratio_j3 = 4 * 5.18  # TODO review gear ratio
home_count_j3 = -740  # TODO calculate home count
max_speed_j3 = 75
# gonna need to update kinematics to account for the joint limits:
# like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
max_positive_angle_j3 = 75  # TODO calculate joint limit
max_negative_angle_j3 = -75  # TODO calculate joint limit
homing_direction_j3 = Stepper.CW

# joint 4
pulse_pin_j4 = 32
dir_pin_j4 = 38
homing_pin_j4 = 40
gear_ratio_j4 = 1 # TODO calculate gear ratio
home_count_j4 = -30 # TODO calculate home count
max_speed_j4 = 50
# gonna need to update kinematics to account for the joint limits:
# like if it says j2 goes to 30 degrees, need to find clockwise alternative for all joints
max_positive_angle_j4 = 90 # TODO calculate joint limits
max_negative_angle_j4 = -40 # TODO calcylate joint limit
homing_direction_j4 = Stepper.CW
