#!/usr/bin/env python3

import numpy as np
import math
import rospy
import time

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
        self.target_pos = None
        self.current_pos = None

    def distance_to_target(self, current_pos, target_pos):
        """
        Calculate the Euclidean distance between the points (X, Y) and (X', Y')
        
        Parameters:
            current_pos ((float), (float)): (X, Y) - Current position
            targer_pos ((float), (float)): (X', Y') - Target position
        Returns:
            euclidean_distance (float): L2 Euclidean distance between the two points
        """
        current_pos = np.array(current_pos)
        target_pos = np.array(target_pos)

        euclidean_distance = np.sqrt(np.power(current_pos - target_pos, 2).sum())
        return euclidean_distance

    def calculate_theta(self, current_pos, target_pos):
        theta = math.atan2( (target_pos[1] - current_pos[1]), (target_pos[0] - current_pos[0]) )
        return theta

    def calculate_velocity(self, theta, dt, rotational_speed, forward_speed):
        """
        Calculate forward speed and angular velocity
        Parameters:
            K (int): Proportional constant
            theta (float): Angle
            dt (float): Iteration time
        
        Returns:
            velocity (float): Velocity  
        """
        velocity = np.array([forward_speed * math.cos(theta + rotational_speed * dt), forward_speed * math.sin(theta + rotational_speed * dt)])
        return velocity

    def control(self, distance_to_target, current_theta, dt, current_pos, target_pos):
        target_theta = self.calculate_theta(current_pos, target_pos)

        theta_error = self.rotational_speed_gain * (target_theta - current_theta)
        distance_error = self.forward_speed_gain * (distance_to_target)

        new_current_pos = self.forward_speed_gain * self.calculate_velocity(target_theta, dt, theta_error, distance_error)

        # 5 degrees = 0.08 rads
        if current_theta == 0 or theta_error >= 5:
            # Stop the robot and correct the angle again
            new_current_pos = 0 
            print(f"theta_error:{theta_error} \t for_speed:{forward_speed} \t new_current_pos{new_current_pos}")
            return target_theta, new_current_pos, theta_error, 0 

        elif theta_error < 5:
            # Forwad speed can be non-zero, while theta orentation still being corrected
            print(f"theta_error:{theta_error} \t for_speed:{distance_error} \t new_current_pos{new_current_pos}")
            return target_theta, new_current_pos, theta_error, distance_error

    def set_current_pos(self, data: Pose_estimation_vectors):
        self.current_pos = np.array([data.translational.x, data.translational.y], dtype=np.float32)
        self.theta = np.float32(data.rotational.z)

        if (self.current_pos is not None) and (self.target_pos is not None):
            self.move_robot()
                
    def set_target_pos(self, data: Pose_estimation_vectors):
        if self.target_pos is None:
            self.target_pos = np.array([data.translational.x, data.translational.y], dtype=np.float32)

    def move_robot(self):
        if (self.current_pos is None) or (self.target_pos is None):
            return False 

        distance_to_target = self.distance_to_target(self.current_pos, self.target_pos)

        if distance_to_target < 0.01:
            print('Target reached!')
            return True
        else:
            # Compute forward and rotation speed with controller
            # set speed to robot
            t_k = self.current_time
            # time.sleep(0.0001)
            self.current_time = time.time()

            dt = self.current_time - t_k

            # Update robot pos with t_k and current_time
            #  target_theta, velocity, theta_error, forward_speed 
            theta, new_current_pos, rotational_speed, forward_speed = self.control(distance_to_target, self.theta, dt, self.current_pos, self.target_pos)

            self.rotational_speed = rotational_speed
            self.forward_speed = forward_speed
            self.theta = theta

            twist = Twist()
            twist.linear.x = forward_speed
            twist.angular.z = rotational_speed 
            pub.publish(twist)

            self.current_pos += new_current_pos

            # print(f"distance_to_target:{distance_to_target} \t forward_speed:{forward_speed} \t rotational_speed:{rotational_speed} ")

if __name__ == '__main__':
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
    rospy.init_node('robot_controller')
    controller = Controller()
    rospy.spin()