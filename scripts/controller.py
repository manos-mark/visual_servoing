#!/usr/bin/env python3

import numpy as np
import math
import rospy
import time
import cv2

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
        self.theta = None
        self.target_homogenious_matrix = None
        self.curr_homogenious_matrix = None

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
        # print('cur depth: ', data.translational.z)
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
        self.theta = data.rotational.z
        self.curr_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))
        # print('\n', 'rotational_matrix\n', rotational_matrix)
        # print('\n', 'translational_vector\n', translational_vector)
        # print('\n', 'homogenious_matrix\n', homogenious_matrix)
        self.theta = np.float32(data.rotational.z)

        if (self.curr_homogenious_matrix is not None) and (self.target_homogenious_matrix is not None):
            self.move_robot()
                
    def set_target_pos(self, data: Pose_estimation_vectors):
        if self.target_homogenious_matrix is None:
            rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
            translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
            homogenious_matrix = np.hstack((rotational_matrix, translational_vector))
            self.target_homogenious_matrix = np.vstack((homogenious_matrix, [0, 0, 0, 1]))

    def move_robot(self):
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
        # print('\n', math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)))
        # print('\n', rotational_matrix)

        # distance_to_target
        rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) # self.distance_to_target(self.current_pos, self.target_pos)

        alpha = normalize(-self.theta + math.atan2(dy, dx))
        beta = normalize(-self.theta - alpha)

        k_rho = 0.3
        k_alpha = 0.8
        k_beta = -0.15
        print('rho: ', rho)
        if rho < 0.01:
            print('Target reached!')
            twist = Twist()
            twist.linear.x = 0
            twist.angular.z = 0
            pub.publish(twist)
            return True
        else:
            v = k_rho * rho
            w = k_alpha * alpha + k_beta * beta
    #         # Compute forward and rotation speed with controller
    #         # set speed to robot
    #         t_k = self.current_time
    #         # time.sleep(0.0001)
    #         self.current_time = time.time()

    #         dt = self.current_time - t_k

    #         # Update robot pos with t_k and current_time
    #         #  target_theta, velocity, theta_error, forward_speed 
    #         theta, new_current_pos, rotational_speed, forward_speed = self.control(distance_to_target, self.theta, dt, self.current_pos, self.target_pos)

    #         self.rotational_speed = rotational_speed
    #         self.forward_speed = forward_speed
    #         self.theta = theta

            twist = Twist()
            twist.linear.x = v
            twist.angular.z = w 
            pub.publish(twist)

    #         self.current_pos += new_current_pos

    #         # print(f"distance_to_target:{distance_to_target} \t forward_speed:{forward_speed} \t rotational_speed:{rotational_speed} ")


def normalize(angle):
    return np.arctan2(np.sin(angle),np.cos(angle))
    
if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('robot_controller')
    controller = Controller()
    rospy.spin()