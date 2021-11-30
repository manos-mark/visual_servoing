#!/usr/bin/env python3

import numpy as np
import math
import rospy
import time
import cv2

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist
from numpy.lib.function_base import _calculate_shapes
from visual_servoing.msg import Pose_estimation_vectors

class Controller:
    def __init__(self):
        rospy.Subscriber('current_position', Pose_estimation_vectors, self.set_current_pos)
        rospy.Subscriber('target_position', Pose_estimation_vectors, self.update_target_position_path)
        self.current_time = 0
        self.target_homogenious_matrix = None
        self.curr_homogenious_matrix = None
        self.theta_current = None
        self.theta_target = None
        self.rho = float("inf")
        self.alpha = float("inf")
        self.beta = float("inf")
        
        self.target_position_path = []

        rospy.on_shutdown(self.stop_robot)
        
        while not rospy.is_shutdown():
            if len(self.target_position_path) == 0:
                self.stop_robot()
                rospy.loginfo('Waiting for path...')
                rospy.sleep(3.0)
            else:
                rospy.loginfo('Robot is moving to the next target!')
                self.move_robot()
        
    def update_target_position_path(self, data):
        self.target_position_path.append(data)

    def set_current_pos(self, data: Pose_estimation_vectors):
        # if self.curr_homogenious_matrix is None:
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
        self.curr_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))
        # print('\n Current Homogemeous: \n',self.curr_homogenious_matrix)
        
        r = R.from_matrix(rotational_matrix)
        self.theta_current = r.as_euler('XYZ', degrees=False)[2]
        # print('theta_current: ', math.degrees(self.theta_current))
        
    def set_target_pos(self, data: Pose_estimation_vectors):
        # if self.target_homogenious_matrix is None:
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
        self.target_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))
        
        r = R.from_matrix(rotational_matrix)
        self.theta_target = r.as_euler('XYZ', degrees=False)[2]
        
        self.rho = float('inf')
        self.alpha = float("inf")
        self.beta = float('inf')

    def move_robot(self):
        target_position_data = self.target_position_path.pop(0)
        self.set_target_pos(target_position_data)

        k_rho = 0.3 # 0.3 (k_rho > 0)
        k_alpha = 0.8 # 0.8 (k_alpha - k_rho > 0)
        k_beta = -0.15 # -0.15 (k_beta < 0)
        constant_vel = 0.3

        while (self.rho>=0.05) and (not rospy.is_shutdown()):

            if self.theta_target is None:
                continue
            # thetag = math.radians(self.theta_target)
            thetag = self.theta_target

            if self.theta_current is None:
                continue
            # theta = math.radians(self.theta_current)
            theta = self.theta_current
            
            if (self.curr_homogenious_matrix is None) or (self.target_homogenious_matrix is None):
                continue

            # print('\n cur_theta: ', theta, '\t\t target_theta: ', thetag)

            t = np.matmul(np.linalg.inv(self.curr_homogenious_matrix), self.target_homogenious_matrix)
            dx = t[0][3]
            dy = t[1][3]

            self.rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2))
            self.alpha = normalize(np.arctan2(dy, dx) - theta)
            self.beta = normalize(thetag - np.arctan2(dy, dx))
            
            v = k_rho * self.rho
            w = k_alpha * self.alpha + k_beta * self.beta
            # print(f'\ntheta: {math.degrees(theta)}, rho: {self.rho}. alpha: {math.degrees(self.alpha)} beta: {math.degrees(self.beta)}')

            print(f'v:{v} \t w:{w}')
            if constant_vel:
                abs_v = abs(v)
                v = v / abs_v * constant_vel
                w = w / abs_v * constant_vel

            self.send_velocity_to_robot(v,w)
            
            rospy.sleep(0.01)
            
        if self.rho < 0.05:
            rospy.loginfo('Target reached!')
            
        self.stop_robot()

    def move_robot_old(self):
        target_position_data = self.target_position_path.pop(0)
        self.set_target_pos(target_position_data) 

        # Controller constants
        k_rho = 0.2
        k_alpha = 0.8
        k_beta = 0.950

        # Fix the initial angle 0.01
        while (abs(self.beta) > 0.09) and (not rospy.is_shutdown()):
            if self.theta_target is None:
                continue

            if self.theta_current is None:
                continue

            if (self.curr_homogenious_matrix is None) or (self.target_homogenious_matrix is None):
                continue

            # print('\n cur_theta: ', theta, '\t\t target_theta: ', thetag)

            t = np.matmul(np.linalg.inv(self.curr_homogenious_matrix), self.target_homogenious_matrix)
            dx = t[0][3]
            dy = t[1][3]

            self.beta = math.atan2(dy, dx)
            
            self.rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) # self.distance_to_target(self.current_pos, self.target_pos)

            v = 0
            w = k_beta * self.beta
            # print('beta: ',self.beta, '\tdx: ',dx, '\tdy: ',dy, '\trho: ', self.rho)
            # print('w: ',w)
            self.send_velocity_to_robot(v,w)
            # beta = float("inf")
        
        # Go to target 0.09
        # while (self.rho >= 0.09) and (abs(self.beta) <= 0.09) and (not rospy.is_shutdown()): 
        while (self.rho >= 0.07) and (not rospy.is_shutdown()): 
            # k_beta = -0.15
            if self.theta_target is None:
                continue
            # thetag = math.radians(self.theta_target)
            thetag = self.theta_target

            if self.theta_current is None:
                continue
            # theta = math.radians(self.theta_current)
            theta = self.theta_current
            
            if (self.curr_homogenious_matrix is None) or (self.target_homogenious_matrix is None):
                return

            # print('\n cur_theta: ', theta, '\t\t target_theta: ', thetag)

            t = np.matmul(np.linalg.inv(self.curr_homogenious_matrix), self.target_homogenious_matrix)

            dx = t[0][3]
            dy = t[1][3]

            self.beta = math.atan2(dy, dx)

            # distance to target
            self.rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) # self.distance_to_target(self.current_pos, self.target_pos)
            # print('beta: ',self.beta, '\tdx: ',dx, '\tdy: ',dy, '\trho: ', self.rho)
 
            v = k_rho * self.rho                
            w = 0
            self.send_velocity_to_robot(v,w)

        # Fix the final angle
        # beta = float("inf")
        # while (abs(beta) > 0.09) and (not rospy.is_shutdown()):
        #     k_beta = 0.950
        #     if self.theta_target is None:
        #         continue
        #     # thetag = math.radians(self.theta_target)
        #     thetag = self.theta_target

        #     if self.theta_current is None:
        #         continue
        #     # theta = math.radians(self.theta_current)
        #     theta = self.theta_current
            
        #     if (self.curr_homogenious_matrix is None) or (self.target_homogenious_matrix is None):
        #         continue

        #     # print('\n cur_theta: ', theta, '\t\t target_theta: ', thetag)

        #     t = np.matmul(np.linalg.inv(self.curr_homogenious_matrix), self.target_homogenious_matrix)
        #     dx = t[0][3]
        #     dy = t[1][3]

        #     rotational_matrix = np.array([
        #                                     [t[0][0], t[0][1], t[0][2]],
        #                                     [t[1][0], t[1][1], t[1][2]],
        #                                     [t[2][0], t[2][1], t[2][2]],
        #                                 ])
            
        #     r = R.from_matrix(rotational_matrix)
        #     beta = r.as_euler('XYZ', degrees=False)[2]

        #     v = 0
        #     w = k_beta * beta
        #     # print('beta: ',beta, '\tdx: ',dx, '\tdy: ',dy)
        #     # print('w: ',w)
        #     self.send_velocity_to_robot(v,w)

        if self.rho < 0.07:
            rospy.loginfo('Target reached!')

    def send_velocity_to_robot(self, v, w):
        twist = Twist()

        # Set maximum and minimum values of turtlebot burger
        if v > 0.2:
            twist.linear.x = 0.2
        elif v < -0.2:
            twist.linear.x = -0.2
        else:
            twist.linear.x = v

        if w > 2.84:
            twist.angular.z = 2.84
        elif w < -2.84:
            twist.angular.z = -2.84
        else:      
            twist.angular.z = w 
        
        # Publish velocity to robot
        try:
            pub.publish(twist)
        except:
            print('Publisher is closed')

        rospy.sleep(0.1)

    def stop_robot(self):
        """
        Publish zero values for angular and linear velocity to /cmd_vel ROS topic, 
        in order to stop the robot
        """
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist) 

        # rospy.sleep(10)
        # self.move_robot()   

def normalize(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))
        
if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('robot_controller')
    controller = Controller()
    rospy.spin()