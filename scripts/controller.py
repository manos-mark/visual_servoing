#!/usr/bin/env python3

import numpy as np
import math
import rospy

from numpy.lib.function_base import _calculate_shapes
from geometry_msgs.msg import Vector3

class Controller:
    def __init__(self):
        self.sub = rospy.Subscriber('current_position', Vector3, move_robot)
        # self.sub = rospy.Subscriber('robot_controller', Vector3[2], move_robot)
    # def __init__(self, forward_speed_gain, rotational_speed_gain):
    #     self.forward_speed_gain = forward_speed_gain
    #     self.rotational_speed_gain = rotational_speed_gain

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

        velocity = self.forward_speed_gain * self.calculate_velocity(target_theta, dt, theta_error, distance_error)

        # 5 degrees = 0.08 rads
        if current_theta == 0 or theta_error >= 5:
            # Stop the robot and correct the angle again
            velocity = 0 
            forward_speed = 0
            print(f"theta_error:{theta_error} \t for_speed:{forward_speed} \t velocity{velocity}")
            return target_theta, velocity, theta_error, forward_speed 

        elif theta_error < 5:
            # Forwad speed can be non-zero, while theta orentation still being corrected
            print(f"theta_error:{theta_error} \t for_speed:{distance_error} \t velocity{velocity}")
            return target_theta, velocity, theta_error, distance_error

def move_robot(data):
    print(data)
#     distance_to_target = self.distance_to_target(self.current_pos, self.target_pos)

#     if distance_to_target < 0.01:
#         print('Target reached!')
#         return False
#     else:
#         # Compute forward and rotation speed with controller
#         # set speed to robot
#         global current_time
#         t_k = current_time
#         time.sleep(iteration_time_sec)
#         current_time = time.time()

#         dt = current_time - t_k

#         # Update robot pos with t_k and current_time
#         global theta
#         theta, velocity, rotational_speed, forward_speed = controller.control(distance_to_target, theta, dt, robot.current_pos, target_pos)

#         robot.rotational_speed = rotational_speed
#         robot.forward_speed = forward_speed
#         robot.update_position(velocity)

#         # print(f"distance_to_target: {distance_to_target} \t dt:{dt} \t  Current Pos:{robot.current_pos} \t Theta error:{theta_error} ")

if __name__ == '__main__':
    rospy.init_node('robot_controller')
    Controller()
    rospy.spin()