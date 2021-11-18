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
        rospy.Subscriber('target_position', Pose_estimation_vectors, self.set_target_pos)
    # def __init__(self, forward_speed_gain, rotational_speed_gain):
        self.forward_speed_gain = 1 # forward_speed_gain
        self.rotational_speed_gain = 1 # rotational_speed_gain
        self.current_time = 0
        self.target_homogenious_matrix = None
        self.curr_homogenious_matrix = None
        self.theta_current = None
        self.theta_target = None

        rospy.on_shutdown(self.stop_robot)
        self.move_robot()
    # def control(self, distance_to_target, current_theta, dt, current_pos, target_pos):
    #     target_theta = self.calculate_theta(current_pos, target_pos)

    #     theta_error = self.rotational_speed_gain * (target_theta - current_theta)
    #     distance_error = self.forward_speed_gain * (distance_to_target)

    #     new_current_pos = self.forward_speed_gain * self.calculate_velocity(target_theta, dt, theta_error, distance_error)

    #     # 5 degrees = 0.08 rads
    #     if current_theta == 0 or theta_error >= 5:
    #         # Stop the robot and correct the angle again
    #         new_current_pos = 0 
    #         print(f"theta_error:{theta_error} \t for_speed:{forward_speed} \t new_current_pos{new_current_pos}")
    #         return target_theta, new_current_pos, theta_error, 0 

    #     elif theta_error < 5:
    #         # Forwad speed can be non-zero, while theta orentation still being corrected
    #         print(f"theta_error:{theta_error} \t for_speed:{distance_error} \t new_current_pos{new_current_pos}")
    #         return target_theta, new_current_pos, theta_error, distance_error

    def set_current_pos(self, data: Pose_estimation_vectors):
        # if self.curr_homogenious_matrix is None:
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
        self.curr_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))
        
        r = R.from_matrix(rotational_matrix)
        self.theta_current = r.as_euler('XYZ', degrees=False)[2]
                
    def set_target_pos(self, data: Pose_estimation_vectors):
        if self.target_homogenious_matrix is None:
            rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
            translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
            homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
            self.target_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))
            
            r = R.from_matrix(rotational_matrix)
            self.theta_target = r.as_euler('XYZ', degrees=False)[2]

    def move_robot(self):
        rho = float("inf")

        while (rho > 0.03) and not rospy.is_shutdown(): 
            if self.theta_target is None:
                continue
            thetag = math.radians(self.theta_target)

            theta = math.radians(self.theta_current)
            if (self.curr_homogenious_matrix is None) or (self.target_homogenious_matrix is None):
                return

            t = np.matmul(self.curr_homogenious_matrix, np.linalg.inv(self.target_homogenious_matrix))
            dx = t[0][3]
            dy = t[1][3]

            rotational_matrix = np.array([
                                            [t[0][0], t[0][1], t[0][2]],
                                            [t[1][0], t[1][1], t[1][2]],
                                            [t[2][0], t[2][1], t[2][2]],
                                        ])

            # distance to target
            rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) # self.distance_to_target(self.current_pos, self.target_pos)

            # angle between X axis and the orientation of the robot
            alpha = normalize(np.arctan2(dy, dx) - theta)
            # alpha = math.degrees(alpha)

            # angle between the orientation of the robot and the target orientation
            beta = normalize(thetag - np.arctan2(dy, dx)) #-theta - alpha
            # beta = math.degrees(beta)
            
            # Controller constants
            k_rho = 0.3
            k_alpha = 0.8
            k_beta = -0.15

            # rho_der = -k_rho * rho * math.cos(alpha)
            # alpha_der = k_rho * math.sin(alpha) - k_alpha * alpha - k_beta * beta
            # beta_der = -k_rho * math.sin(alpha)

            # Publish zero velocities when the distance to target is less than the distance error
            print(f'\ntheta: {theta}, rho: {rho}. alpha: {alpha} beta: {beta}')
            
            v = k_rho * rho                

            w = (k_alpha * alpha + k_beta * beta)
            # if alpha < 0:
            #     w = -w

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

        self.stop_robot()

        if rho < 0.03:
            print('Target reached!')

    def stop_robot(self):
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)    

def normalize(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))
        
if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('robot_controller')
    controller = Controller()
    rospy.spin()