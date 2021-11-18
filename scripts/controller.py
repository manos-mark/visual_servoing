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
        self.current_time = 0
        self.target_homogenious_matrix = None
        self.curr_homogenious_matrix = None
        self.theta = None
        self.theta_tar = None

    def set_current_pos(self, data: Pose_estimation_vectors):
        # if self.curr_homogenious_matrix is None:
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
        self.curr_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))
        r = R.from_matrix(rotational_matrix)
        self.theta = r.as_euler('XYZ', degrees=False)[2]
        # print(f"Theta :  {self.theta}")
                
    def set_target_pos(self, data: Pose_estimation_vectors):
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
        self.target_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))
        r = R.from_matrix(rotational_matrix)
        self.theta_tar = r.as_euler('XYZ', degrees=False)[2]

    def move_robot(self):
        
        rate = rospy.Rate(10)
        while not rospy.is_shutdown(): 
            if (self.curr_homogenious_matrix is None) or (self.target_homogenious_matrix is None):
                continue 
            
            # print(f'Current_homogenious: \n{self.curr_homogenious_matrix} \n Target_homogenious: \n{self.target_homogenious_matrix}')

            t = np.matmul(self.curr_homogenious_matrix, np.linalg.inv(self.target_homogenious_matrix))
            dx = t[0][3]
            dy = t[1][3]

            rotational_matrix = np.array([
                                            [t[0][0], t[0][1], t[0][2]],
                                            [t[1][0], t[1][1], t[1][2]],
                                            [t[2][0], t[2][1], t[2][2]],
                                        ])
            
            theta = self.theta #cv2.Rodrigues(rotational_matrix)[0][2]
            # distance to target
            theta_error = theta - self.theta_tar
            rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) # self.distance_to_target(self.current_pos, self.target_pos)
            # angle between X axis and the orientation of the robot
            alpha = -theta + math.atan2(dx, dy) #normalize(-theta + math.atan2(dy, dx))
            # angle between the orientation of the robot and the target orientation
            beta = -theta_error - alpha#normalize(-theta - alpha)
            
            if rho < 0.01:
                print('Target reached!')
                twist = Twist()
                twist.linear.x = 0
                twist.angular.z = 0
                pub.publish(twist)
                break
            
            # Controller constants
            k_rho = 0.3
            k_alpha = 0.8
            k_beta = -0.15

            # rho_der = -k_rho * rho * math.cos(alpha)
            # alpha_der = k_rho * math.sin(alpha) - k_alpha * alpha - k_beta * beta
            # beta_der = -k_rho * math.sin(alpha)

            # rho = rho_der
            # alpha = alpha_der
            # beta = beta_der

            # Publish zero velocities when the distance to target is less than the distance error
            print(f'\ntheta: {theta}, rho: {rho}. alpha: {alpha} beta: {beta}')

            v = k_rho * rho                

            w = -(k_alpha * alpha + k_beta * beta)
            if alpha < 0:
                w = -w

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
            pub.publish(twist)
            rate.sleep()

        

        print('Target reached!')
        twist = Twist()
        twist.linear.x = 0
        twist.angular.z = 0
        pub.publish(twist)
        
if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('robot_controller')
    controller = Controller()
    
    controller.move_robot()
    rospy.spin()