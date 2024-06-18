#!/usr/bin/env python3

import threading
import numpy as np
from math import radians, degrees, sqrt, pi, asin, cos, sin
from stepper import Stepper
from scipy.optimize import minimize
import time
from config import *

class Arm:
    
    def __init__(self, j1, j2, j3):
        
        self.joints = [j1, j2, j3]
        self.joint_angles = [0, 0, 0]

        self.camera_transformation = np.array([[ 0,-0.4067, 0.9135, 0.122497],
                                               [-1, 0,      0,      0.032445],
                                               [ 0,-0.9135,-0.4067, 0.416466],
                                               [ 0, 0,      0,      1]])

        # Parameters
        self.L1 = 126.40
        self.L2 = 46
        self.L3 = 150
        self.L4 = 15
        self.L5 = 194.10
        self.y_offset = -2.70
        # TODO add x offsets to acount for errors in manufacturing process

        self.x_offset = 3
        self.link_lengths = [self.L1, self.L2, self.L3, self.L4, self.L5]
        self.bounds = [(-pi/2, pi/3), (-pi/18, 2*pi/3), (-4*pi/9, 4*pi/9)]

        # we are keeping everything in radians and mm. for printing angles we use degrees since
        # it is easier.
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
    def fk(self, thetas):
        theta_1, theta_2, theta_3 = thetas
        L1, L2, L3, L4, L5 = self.link_lengths
        y_offset = self.y_offset
        x_offset = self.x_offset

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
        base_ee_trans[0][3] += x_offset
        return base_ee_trans
 

    # Calculate inverse kinematics using L-BFGS-B
    def ik(self, desired_ee, initial_guess=np.array([pi/4, pi/4, pi/4]), max_iter=1000, tol=1e-3):
        # Objective function
        def objective(thetas, target_position):
            fk_result = self.fk(thetas)
            ee_position = fk_result[:3, 3]  # Extract the end-effector position
            error = np.linalg.norm(ee_position - target_position)  # Compute the Euclidean distance
            return error

        options = {
        'maxiter': max_iter,
        'disp': False,  # Display convergence messages
        }
        
        start = time.time()
        result = minimize(objective, initial_guess, args=(target_position), method='L-BFGS-B', bounds=self.bounds, options=options, tol=tol)
        end = time.time()

        elapsed_time = end - start
        return result.x, result, elapsed_time

    def trajectory_planning(self, theta_end):
        
        tf = 5     # time of motion [ s ]
        N = 4      # number of points in trajectory
        method = 5 # time-scaling method (quintic)
        theta_start = self.get_current_theta()
        joint_traj = Arm.JointTrajectory(theta_start, theta_end, tf, N, method)
        return joint_traj
            
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
            time.sleep(2)

    def write_joints(self, joint_pose):
        zipped = zip(self.joints, joint_pose)
        threads = []
        updated_angle = []
        
        for joint, joint_angle in zipped:
            thread = threading.Thread(target=self.write_joint, args=(joint, joint_angle))
            thread.start()
            threads.append(thread)
            #updated_joint_angle = self.write_joint(joint, joint_angle) # one joint at a time

        # wait for each joint to reach its position
        for thread in threads:
            updated_joint_angle = thread.join()
            updated_angle.append(updated_joint_angle)
            threads.remove(thread) # remove thread after completion
        
        print(updated_angle)
        self.update_angles(updated_angle)

    def write_joint(self, joint, joint_angle):
        # TODO make write in stepper library return the actual angle of the joint
        # because of our step angle resolution there is error --> this will help account for the error in pose
        angle_deg = degrees(joint_angle)
        print("angle for joint: ", joint.id, " ", angle_deg)
        
        updated_angle = joint.write(angle_deg)
        
        return updated_angle # writes angle to joint --> needs to be threading though
    def update_angles(self, joint_angles):
        
        # updates our joint angles to the ones actually achieved by the motors
        for x in range(len(joint_angles)):
            self.joint_angles[x] = joint_angles[x]

    # all joint angles are in radians
    def get_current_theta(self):

        return self.joint_angles

    def print_joint_angles(self):

        for joint, angle in enumerate(self.joint_angles):
            print("J",joint, ":", degrees(angle))
    
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

                    
    def run(self):
        pass
    
    def cleanup(self):
        for joint in self.joints:
            joint.cleanup()

    @staticmethod
    def CubicTimeScaling(Tf, t):
        """Computes s(t) for a cubic time scaling

        :param Tf: Total time of the motion in seconds from rest to rest
        :param t: The current time t satisfying 0 < t < Tf
        :return: The path parameter s(t) corresponding to a third-order
             polynomial motion that begins and ends at zero velocity

        Example Input:
            Tf = 2
            t = 0.6
        Output:
            0.216
        """
        return 3 * (1.0 * t / Tf) ** 2 - 2 * (1.0 * t / Tf) ** 3

    @staticmethod
    def QuinticTimeScaling(Tf, t):
        """Computes s(t) for a quintic time scaling

        :param Tf: Total time of the motion in seconds from rest to rest
        :param t: The current time t satisfying 0 < t < Tf
        :return: The path parameter s(t) corresponding to a fifth-order
             polynomial motion that begins and ends at zero velocity and zero
             acceleration

        Example Input:
            Tf = 2
            t = 0.6
        Output:
            0.16308
        """
        return 10 * (1.0 * t / Tf) ** 3 - 15 * (1.0 * t / Tf) ** 4 \
           + 6 * (1.0 * t / Tf) ** 5
    
    @staticmethod
    def JointTrajectory(thetastart, thetaend, Tf, N, method):
        """Computes a straight-line trajectory in joint space

        :param thetastart: The initial joint variables
        :param thetaend: The final joint variables
        :param Tf: Total time of the motion in seconds from rest to rest
        :param N: The number of points N > 1 (Start and stop) in the discrete
              representation of the trajectory
        :param method: The time-scaling method, where 3 indicates cubic (third-
                   order polynomial) time scaling and 5 indicates quintic
                   (fifth-order polynomial) time scaling
        :return: A trajectory as an N x n matrix, where each row is an n-vector
             of joint variables at an instant in time. The first row is
             thetastart and the Nth row is thetaend . The elapsed time
             between each row is Tf / (N - 1)

        Example Input:
            thetastart = np.array([1, 0, 0, 1, 1, 0.2, 0,1])
            thetaend = np.array([1.2, 0.5, 0.6, 1.1, 2, 2, 0.9, 1])
            Tf = 4
            N = 6
            method = 3
        Output:
            np.array([[     1,     0,      0,      1,     1,    0.2,      0, 1]
                        [1.0208, 0.052, 0.0624, 1.0104, 1.104, 0.3872, 0.0936, 1]
                        [1.0704, 0.176, 0.2112, 1.0352, 1.352, 0.8336, 0.3168, 1]
                        [1.1296, 0.324, 0.3888, 1.0648, 1.648, 1.3664, 0.5832, 1]
                        [1.1792, 0.448, 0.5376, 1.0896, 1.896, 1.8128, 0.8064, 1]
                        [   1.2,   0.5,    0.6,    1.1,     2,      2,    0.9, 1]])
        """
        N = int(N)
        timegap = Tf / (N - 1.0)
        traj = np.zeros((len(thetastart), N))
        for i in range(N):
            if method == 3:
                s = Arm.CubicTimeScaling(Tf, timegap * i)
            else:
                s = Arm.QuinticTimeScaling(Tf, timegap * i)
            traj[:, i] = s * np.array(thetaend) + (1 - s) * np.array(thetastart)
        traj = np.array(traj).T
        return traj

if __name__ == '__main__':
 
    try:
        print("setting up the arm")
        j1 = Stepper(pulse_pin_j1, dir_pin_j1, enable_pin, homing_pin_j1, pulses_per_rev_j1, gear_ratio_j1, max_speed_j1,max_positive_angle_j1,max_negative_angle_j1, home_count_j1,homing_direction_j1, stepper_id =1, debug=False) 
        j2 = Stepper(pulse_pin_j2, dir_pin_j2, enable_pin, homing_pin_j2, pulses_per_rev_j2, gear_ratio_j2, max_speed_j2,max_positive_angle_j2, max_negative_angle_j2,home_count_j2,homing_direction_j2 ,inverted=True, stepper_id=2, debug=True)
        j3 = Stepper(pulse_pin_j3, dir_pin_j3, enable_pin, homing_pin_j3, pulses_per_rev_j3, gear_ratio_j3, max_speed_j3,max_positive_angle_j3, max_negative_angle_j3,home_count_j3,homing_direction_j3, stepper_id = 3, debug = False)
       
        arm = Arm(j1, j2, j3)
        arm.home()

        #print(arm.fk([0, 0, 0]))
        # Target end-effector position (example)
        target_position = np.array([200, 70, 0])
        # Calculate inverse kinematics using L-BFGS-B
        optimized_thetas, result, elapsed_time = arm.ik(target_position)
        [print(degrees(x)) for x in optimized_thetas]
        joe = input("shall we continue?")
        traj = arm.trajectory_planning(optimized_thetas)
        print(traj)
        arm.follow_trajectory(traj)
       #print(arm.joint_angles)

        #print("Optimized joint angles (radians):", [round(x, 4) for x in optimized_thetas])
        #print("Optimization success:", result.success)
        #print("Message:", result.message)
        #print(f"Elapsed time: {elapsed_time:.4f} seconds")
        
        # get joint angles
        # joint_angles = arm.ik(desired_ee)

        # get trajectory to follow
        # traj = arm.trajectory_planning(joint_angles)

        # follow the trajectory
        # arm.follow_trajectory(traj)

    except KeyboardInterrupt:
        j1.cleanup()
