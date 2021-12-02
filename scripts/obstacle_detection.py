#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
# from mira_sensors import MiraSensors
from geometry_msgs.msg import Point
import path_planning 


class ObstacleTracker(object):
    """Obstacle tracker class
    """

    def __init__(self, hsv_min, hsv_max, robot_size_in_pixels):
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max
        self.offset = robot_size_in_pixels

    def convert_center_to_pixels(self, image, shortest_path):
        """Convert every middle point of the path from indexes to pixels

        :param image: Input image
        :type image: ndarray
        :param shortest_path: Shortest path indexes
        :type shortest_path: list
        :return: Shortest path pixels
        :rtype: list
        """
        if shortest_path is None:
            return

        rows = image.shape[0]
        cols = image.shape[1]
                
        offset = self.offset

        rows_count = int(rows/offset)
        cols_count = int(cols/offset)

        rows = rows_count*offset
        cols = cols_count*offset
        
        shortest_path_center_pixels = [0] * len(shortest_path) #np.zeros_like(shortest_path)

        for (i, col) in enumerate(range(0, cols, offset)):
            for (j, row) in enumerate(range(0, rows, offset)):
                
                center = (row+int(offset/2), col+int(offset/2))

                for (index, value) in enumerate(shortest_path):
                    if (j,i) == value:
                        shortest_path_center_pixels[index] = center

        return shortest_path_center_pixels

    def draw_map(self,
                    image,              #- Input image
                    obstacles_map,
                    shortest_path=None,
                    start_index=None, 
                    goal_index=None,
                    line=1,             #- line's thickness
                    imshow=False        #- show the image
                ):
        """Draw the map with obstacles and shortest path

        :param image: Input image
        :type image: ndarray
        :param obstacles_map: Obstacles map
        :type obstacles_map: list
        :param shortest_path: Shortest path, defaults to None
        :type shortest_path: list, optional
        :param start_index: Start index, defaults to None
        :type start_index: tuple, optional
        :param goal_index: Goal index, defaults to None
        :type goal_index: tuple, optional
        """

        obstacles_map = np.array(obstacles_map).T
        obstacles_map = obstacles_map.tolist()

        cols = image.shape[0]
        rows = image.shape[1]
        
        offset = self.offset
        
        rows_count = int(rows/offset)
        cols_count = int(cols/offset)

        rows = rows_count*offset
        cols = cols_count*offset
        
        for i, row in enumerate(range(0, rows+offset, offset)):
            # if i <= rows_count:
            image = cv2.line(image, (0, row), (cols_count*offset, row), (0,0,0), line)

        for i, col in enumerate(range(0, cols+offset, offset)):
            # if i <= cols_count:  
            image = cv2.line(image, (col, 0), (col, rows_count*offset), (0,0,0), line)

        for (i, col) in enumerate(range(offset, cols+offset, offset)):
            for (j, row) in enumerate(range(0, rows, offset)):
                if obstacles_map is not None and i <= len(obstacles_map)-1 and j <= len(obstacles_map)-1:
                    is_obstacle = obstacles_map[i][j]
                    
                    text = 'X' if is_obstacle else 'O' 
                    color = (255,255,0) if is_obstacle else (255,0,0)
                    
                    center = (row+int(offset/2), col-int(offset/2))
                    text_center = (row+int(offset/3), col-int(offset/3))

                    image = cv2.putText(image, text, text_center, cv2.FONT_HERSHEY_SIMPLEX, 
                        1, color, 3, cv2.LINE_AA)

                    if start_index is not None:
                        if (j,i) == start_index:
                                color = (255, 255, 255)
                                cv2.putText(image, 'S', text_center, cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, color, 5, cv2.LINE_AA)
                            
                    if goal_index is not None:
                        if (j,i) == goal_index:
                            color = (255, 255, 255)
                            cv2.putText(image, 'G', text_center, cv2.FONT_HERSHEY_SIMPLEX, 
                                1, color, 5, cv2.LINE_AA)
                
                #Radius of circle
                radius = 6
                # Orange color in BGR
                color = (0, 140, 255)
                # Line thickness of 2 px
                thickness = -1

                if shortest_path is not None:
                    for (index, value) in enumerate(shortest_path):
                        if (j,i) == value:
                            cv2.circle(image, center, radius, color, thickness)    
                            # shortest_path_center_pixels[index] = center

        if imshow:
            # Show keypoints
            cv2.imshow("Map", image)
        # rospy.loginfo('Drawing map completed')
        return(image)

    def generate_map(self, image, cur_pos_center, goal_pos_center):
        """Getting the current and target position pixels, generate a map of obstacles
        and return the map and the corresponding indexes of the current and target points.

        :param image: Input image
        :type image: ndarray
        :param cur_pos_center: Current position center pixels
        :type cur_pos_center: tuple
        :param goal_pos_center: Target position center pixels
        :type goal_pos_center: tuple

        :return obstacles_map: Obstacles map 
        :rtype obstacles_map: list
        :return cur_pos_center_indexes: Current position center indexes
        :rtype cur_pos_center_indexes: tuple
        :return goal_pos_center_indexes: Target position center indexes
        :rtype goal_pos_center_indexes: tuple
        """
        # rospy.loginfo('Detecting obstacles started')

        obstacles_map = []

        rows = image.shape[0]
        cols = image.shape[1]

        cur_pos_center_indexes = goal_pos_center_indexes = None

        # Robot size - should be equal with the box size
        offset = self.offset
        
        rows_count = int(rows/offset)
        cols_count = int(cols/offset)

        rows = rows_count*offset
        cols = cols_count*offset
        
        for (i, row) in enumerate(range(0, rows, offset)):
            row_list = []
            for (j, col) in enumerate(range(0, cols, offset)):
                box = image[col:col+offset, row:row+offset]
                is_obstacle = self.does_image_contain_obstacles(box)
                row_list.append(1 if is_obstacle else 0)

                if cur_pos_center is not None:
                    x = cur_pos_center[0]
                    y = cur_pos_center[1]
                    if (col <= x) and (x <= col+offset) and (row <= y) and (y <= row+offset+offset):
                        cur_pos_center_indexes = (j,i)

                if goal_pos_center is not None:
                    x = goal_pos_center[0]
                    y = goal_pos_center[1]
                    if (col <= x) and (x <= col+offset) and (row <= y) and (y <= row+offset+offset):
                        goal_pos_center_indexes = (j,i)

            obstacles_map.append(row_list)

        return obstacles_map, cur_pos_center_indexes, goal_pos_center_indexes

    def does_image_contain_obstacles(self, image):
        """Check if image contain obstacles, by obstacle we describe every pixel 
        with a specific color, in this case is RED

        :param image: Input image
        :type image: ndarray
        :return: True if image contains obstacle otherwise False
        :rtype: bool
        """
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = self.hsv_min#np.array([0,0,245]) # b,g,r values
        upper = self.hsv_max#np.array([20,20,255]) # b,g,r values

        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)

        return True if 255 in output else False

