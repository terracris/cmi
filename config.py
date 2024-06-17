from stepper import Stepper

pulses_per_rev = 1600
home_count_multiplier = pulses_per_rev / 200
enable_pin = 37

# joint 1
pulse_pin_j1 = 11
dir_pin_j1 = 13
homing_pin_j1 = 15
gear_ratio_j1 = 2.4
home_count_j1 = -93 * home_count_multiplier 
max_speed_j1 = 75     # need to tune
max_positive_angle_j1 = 60
max_negative_angle_j1 = -90
homing_direction_j1 = Stepper.CCW

# joint 2
pulse_pin_j2 = 29
dir_pin_j2 = 31
homing_pin_j2 = 33
gear_ratio_j2 = 2.4 * 5.18
home_count_j2 = -110 * home_count_multiplier
max_speed_j2 = 80   
max_positive_angle_j2 = 120
max_negative_angle_j2 = -10
homing_direction_j2 = Stepper.CCW

# joint 3
pulse_pin_j3 = 36
dir_pin_j3 = 38
homing_pin_j3 = 40
gear_ratio_j3 = 2.4 * 5.18  
home_count_j3 = -390 * home_count_multiplier # need to tune 
max_speed_j3 = 75
max_positive_angle_j3 = 80  
max_negative_angle_j3 = -80  
homing_direction_j3 = Stepper.CW

