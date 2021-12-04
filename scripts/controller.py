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
    """Robot controller class using current and target homogeneous matrices
    """
    def __init__(self):
        rospy.init_node('robot_controller')
        self.velocity_publisher = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        rospy.Subscriber('current_position', Pose_estimation_vectors, self.set_current_pos)
        rospy.Subscriber('target_position', Pose_estimation_vectors, self.update_target_position_path)
        
        self.current_time = 0
        self.target_homogeneous_matrix = None
        self.curr_homogeneous_matrix = None
        self.rho = float("inf")
        self.alpha = float("inf")
        self.beta = float("inf")
        
        self.target_position_path = []

        rospy.on_shutdown(self.stop_robot)
        
        i = 1
        while not rospy.is_shutdown():
            if len(self.target_position_path) == 0:
                self.stop_robot()
                rospy.loginfo('Waiting for path...')
                rospy.sleep(3.0)
            else:
                rospy.loginfo(f'Robot is moving to target with index {i}!')
                self.move_robot()
                i += 1
        
        rospy.spin()
        
    def update_target_position_path(self, data):
        """Update the target position path for each middlepoint

        :param data: Rotational and translational vectors
        :type data: Pose_estimation_vectors
        """
        self.target_position_path.append(data)

    def set_current_pos(self, data: Pose_estimation_vectors):
        """Calculate the rotational matrix from the rotational vector and apply some transformations
        to get the homogeneous matrix and set it as the current position. Furthermore we calculate
        theta using Euler angles.

        :param data: Rotational and translational vectors
        :type data: Pose_estimation_vectors
        """
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogeneous_matrix = np.hstack((rotational_matrix, translational_vector))
        self.curr_homogeneous_matrix = np.vstack((homogeneous_matrix, [0, 0, 0, 1]))
        
    def set_target_pos(self, data: Pose_estimation_vectors):
        """Calculate the rotational matrix from the rotational vector and apply some transformations
        to get the homogeneous matrix and set it as the target position. Furthermore we calculate
        theta using Euler angles.

        :param data: Rotational and translational vectors
        :type data: Pose_estimation_vectors
        """
        
        rotational_matrix, _ = cv2.Rodrigues(np.array([data.rotational.x, data.rotational.y, data.rotational.z], dtype=np.float32))
        translational_vector = np.array([[data.translational.x], [data.translational.y], [data.translational.z]], dtype=np.float32)
        homogeneous_matrix = np.hstack((rotational_matrix, translational_vector))
        self.target_homogeneous_matrix = np.vstack((homogeneous_matrix, [0, 0, 0, 1]))
        
        self.rho = float('inf')
        self.alpha = float("inf")
        self.beta = float('inf')

    def move_robot(self):
        """Fix the angle of the current position according to the next position,
        then move the robot to it, and finally perform parking.
        """
        target_position_data = self.target_position_path.pop(0)
        self.set_target_pos(target_position_data) 
        
        target_reached = False

        # Controller constants
        k_rho = 0.6
        k_beta = 0.950
        k_alpha = 0.650

        while (not target_reached) and (not rospy.is_shutdown()):
            # Fix the initial angle 
            while (abs(self.alpha) > 0.07) and (not rospy.is_shutdown()):
                if (self.curr_homogeneous_matrix is None) or (self.target_homogeneous_matrix is None):
                    continue

                t = np.matmul(np.linalg.inv(self.curr_homogeneous_matrix), self.target_homogeneous_matrix)
                dx = t[0][3]
                dy = t[1][3]

                self.alpha = math.atan2(dy, dx)
                
                self.rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) # self.distance_to_target(self.current_pos, self.target_pos)

                v = 0
                w = k_alpha * self.alpha
                self.send_velocity_to_robot(v,w)
            
            # Go to target 
            while (self.rho >= 0.08) and (not rospy.is_shutdown()): 
                if (self.curr_homogeneous_matrix is None) or (self.target_homogeneous_matrix is None):
                    return

                # print('\n cur_theta: ', theta, '\t\t target_theta: ', thetag)

                t = np.matmul(np.linalg.inv(self.curr_homogeneous_matrix), self.target_homogeneous_matrix)

                dx = t[0][3]
                dy = t[1][3]

                self.alpha = math.atan2(dy, dx)

                # distance to target
                self.rho = math.sqrt(math.pow(dy, 2) + math.pow(dx, 2)) # self.distance_to_target(self.current_pos, self.target_pos)
    
                v = k_rho * self.rho                
                w = 0
                self.send_velocity_to_robot(v,w)

            # Fix the final angle
            while (abs(self.beta) > 0.09) and (len(self.target_position_path)==0) and (not rospy.is_shutdown()):
                k_beta = 0.950
                
                if (self.curr_homogeneous_matrix is None) or (self.target_homogeneous_matrix is None):
                    continue

                # print('\n cur_theta: ', theta, '\t\t target_theta: ', thetag)

                t = np.matmul(np.linalg.inv(self.curr_homogeneous_matrix), self.target_homogeneous_matrix)
                dx = t[0][3]
                dy = t[1][3]

                rotational_matrix = np.array([
                                                [t[0][0], t[0][1], t[0][2]],
                                                [t[1][0], t[1][1], t[1][2]],
                                                [t[2][0], t[2][1], t[2][2]],
                                            ])
                
                r = R.from_matrix(rotational_matrix)
                self.beta = r.as_euler('XYZ', degrees=False)[2]

                v = 0
                w = k_beta * self.beta

                self.send_velocity_to_robot(v,w)

            if (self.rho < 0.08):# and (abs(self.beta) < 0.07):
                rospy.loginfo('Target reached!')
                target_reached = True
                break

    def send_velocity_to_robot(self, v, w):
        """Publish angular and linear velocities to the robot, 
        but first apply some speed limitations.

        :param v: Linear velocity
        :type v: float
        :param w: Angular velocity
        :type w: float
        """
        twist = Twist()
        # Set maximum and minimum values of turtlebot burger
        twist.linear.x = max(v, -0.2) if v < 0 else min(v, 0.2)
        twist.angular.z = max(w, -2.84) if w < 0 else min(w, 2.84)
        
        # Publish velocity to robot
        try:
            self.velocity_publisher.publish(twist)
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
        self.velocity_publisher.publish(twist) 


if __name__ == '__main__':
    controller = Controller()