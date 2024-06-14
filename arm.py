#!/usr/bin/env python3

import threading
import numpy as np
from math import radians, degrees, sqrt, atan2, pi, asin, cos
# import modern_robotics as mr
from stepper import Stepper
from time import sleep
from config import *

class Arm:
    
    def __init__(self, j1, j2, j3):
        
        self.joints = [j1, j2, j3]
        self.joint_angles = [0, 0, 0]

        self.camera_transformation = np.array([[ 0,-0.4067, 0.9135, 0.122497],
                                               [-1, 0,      0,      0.032445],
                                               [ 0,-0.9135,-0.4067, 0.416466],
                                               [ 0, 0,      0,      1]])

        # EE orientation error tol
        self.eomg = 0.01 
        # EE position error tol --> Tolerance is 1mm
        self.ev = 0.01

        # self.home()

    def home(self):
        # joint is the actual stepper motor class

        print("going to home")
        threads = []

        for joint in self.joints:
            thread = threading.Thread(target=self.home_joint, args=(joint,))
            thread.start()
            threads.append(thread)

        # wait for each joint to home
        for thread in threads:
            thread.join()
            threads.remove(thread) # remove thread after completion
        
        print("All threads finished")

    # blocking function to home each joint
    def home_joint(self, joint):
        print("homing joint")
        joint.home()

    # theta_list is list of joint angles 
    def fk(self, theta_list):
        pass

    def ik(self, desired_ee):
        pass 

    def trajectory_planning(self, ik):
        
        tf = 5     # time of motion [ s ]
        N = 5      # number of points in trajectory
        method = 5 # time-scaling method (quintic)
        theta_start = self.get_current_theta()
        #joint_traj = mr.JointTrajectory(theta_start, ik, tf, N, method)
        return None
    
    def follow_trajectory(self, trajecory):
        start_time = Stepper.get_time()  # time resolution will be in seconds
        # trajectory is a numpy array size (N x J)
        # N: Number of points in the trajectory
        # J: Number of joints in the robot arm
        num_trajecory_points = trajecory.shape[0]

        # ALWAYS SKIP THE FIRST TRAJECTORY --> you are already there.

        for point in range(1, num_trajecory_points):
            joint_pose = trajecory[point]
            self.write_joints(joint_pose)

    def write_joints(self, joint_pose):
        zipped = zip(self.joints, joint_pose)
        threads = []
        updated_angle = []
        
        for joint, joint_angle in zipped:
            thread = threading.Thread(target=self.write_joint, args=(joint, joint_angle))
            thread.start()
            threads.append(thread)

        # wait for each joint to reach its position
        for thread in threads:
            updated_joint_angle = thread.join()
            updated_angle.append(updated_joint_angle)
            threads.remove(thread) # remove thread after completion
        
        self.update_angles(updated_angle)

    def write_joint(self, joint, joint_angle):
        # TODO make write in stepper library return the actual angle of the joint
        # because of our step angle resolution there is error --> this will help account for the error in pose
        angle_deg = degrees(joint_angle)
        #print("angle for joint: ", joint.id, " ", angle_deg)
        
        updated_angle = joint.write(angle_deg)
        
        return updated_angle # writes angle to joint --> needs to be threading though

    def update_angles(self, joint_angles):
        
        # updates our joint angles to the ones actually achieved by the motors
        for x in range(len(joint_angles)):
            self.joint_angles[x] = joint_angles[x]

    def get_current_theta(self):

        # returns our current joint angles in radians
        theta_list = []

        for theta in self.joint_angles:
            print(theta)
            theta_list.append(radians(theta))
        
        return theta_list
    
    # receives GetPlan message
    def pickup(self, msg):
        
        if self.is_active:
            empty_path = Path()
            empty_path.poses = []
            return empty_path

        self.is_active = True
        print("yoooooo, we got a request")
        goal = msg.goal.pose.position
        
        x, y, z = goal.x, goal.y, goal.z

        print("x: ", x, "y: ", y, "z: ", z)

        # create (4x1) numpy array
        camera_point = np.array([x, y, z, 1]).T
        desired_ee_from_arm = np.dot(self.camera_transformation, camera_point) 
        trans_x, trans_y, trans_z = desired_ee_from_arm[0], desired_ee_from_arm[1], desired_ee_from_arm[2]
        
        print("trans_x: ", trans_x, "trans_y: ", trans_y, "trans_z: ", trans_z)
        x_offset = 0.07 # 7cm
        offset_x = trans_x - x_offset

        l1, l2, l3 = 0.53477, 0.37063, 0.559
        alpha = asin(x_offset/l1)
        s = trans_z - (l1*cos(alpha))
        r = sqrt((offset_x**2) + (trans_y**2))
        c3 = (r**2 + s**2 - l2**2 - l3**2) / (2*(l1*cos(alpha))*l2)
        s3 = -sqrt(1-(c3**2))
        gamma = atan2(s, r)
        phi = atan2((l3*s3), (l2+ (l3*c3)))

        #j1 = atan2(trans_y, trans_x)
        #j2 = (gamma - phi) + (pi/2)
        #j3 = atan2(s3, c3) + (pi/2)

        j1, j2, j3 = ik_geo(trans_x, trans_y, trans_z)
        if j1 < 0:
            j1 = j1*1.3
        j2 = j2*1
        j3 = j3*1.10
        joint_angles = [j1, j2, j3]
        
        print()
        [print("joint ", (joint+1),": ", degrees(angle), "degrees") for joint, angle in enumerate(joint_angles) ]

        traj = self.trajectory_planning(joint_angles)

        print("trajectory angles: ", traj)
        
        self.home()
        self.follow_trajectory(traj)

        poses = []

            
    def run(self):
        pass
    
    def cleanup(self):
        for joint in self.joints:
            joint.cleanup()
            

if __name__ == '__main__':
 
    try:
        print("setting up the arm")
        j1 = Stepper(pulse_pin_j1, dir_pin_j1, enable_pin, homing_pin_j1, pulses_per_rev, gear_ratio_j1, max_speed_j1,max_positive_angle_j1,max_negative_angle_j1, home_count_j1,homing_direction_j1, stepper_id =1, debug=True) 
        j2 = Stepper(pulse_pin_j2, dir_pin_j2, enable_pin, homing_pin_j2, pulses_per_rev, gear_ratio_j2, max_speed_j2,max_positive_angle_j2, max_negative_angle_j2,home_count_j2,homing_direction_j2 ,inverted=True, stepper_id=2, debug=False)
        j3 = Stepper(pulse_pin_j3, dir_pin_j3, enable_pin, homing_pin_j3, pulses_per_rev, gear_ratio_j3, max_speed_j3,max_positive_angle_j3, max_negative_angle_j3,home_count_j3,homing_direction_j3,kp=0.10,kd=0.003, stepper_id = 3)
       
        arm = Arm(j1, j2, j3)
        arm.home()

        
        # get joint angles
        # joint_angles = arm.ik(desired_ee)

        # get trajectory to follow
        # traj = arm.trajectory_planning(joint_angles)

        # follow the trajectory
        # arm.follow_trajectory(traj)

    except KeyboardInterrupt:
        j1.cleanup()
