#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
# from mira_sensors import MiraSensors
from geometry_msgs.msg import Point
import path_planning 


class ObstacleTracker(object):

    def __init__(self, hsv_min, hsv_max, robot_size_in_pixels):
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max
        self.offset = robot_size_in_pixels

    def convert_center_to_pixels(self, image, window_adim, shortest_path):
        if shortest_path is None:
            return
            
        if window_adim: 
            image = self.crop_image(image, window_adim)

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
                    window_adim,        #- window in adimensional units
                    shortest_path=None,
                    start_index=None, 
                    goal_index=None,
                    color=(255,0,0),    #- line's color
                    line=1,             #- line's thickness
                    imshow=False        #- show the image
                ):
        """
        Draw search window: returns the image
        return(image)
        """
        # rospy.loginfo('Drawing map started')

        obstacles_map = np.array(obstacles_map).T
        obstacles_map = obstacles_map.tolist()
        # for i in obstacles_map:
        #     print(i)
        # print('\n')
        if window_adim:
            image = self.crop_image(image, window_adim)

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

    def get_relative_position(self, image, keyPoint):
        """
        Obtain the camera relative frame coordinate of one single keypoint
        return(x,y)
        """
        rows = float(image.shape[0])
        cols = float(image.shape[1])
        
        center_x    = 0.5*cols
        center_y    = 0.5*rows

        x = (keyPoint[0] - center_x)/(center_x)
        y = (keyPoint[1] - center_y)/(center_y)
        
        return y,x

    def get_absolute_position(self, image, keyPoint):
        """
        Obtain the camera relative frame coordinate of one single keypoint
        return(x,y)
        """
        x = int(keyPoint[0])
        y = int(keyPoint[1])
        size = int((keyPoint.size/2))
        
        return x, y, size
    
    def crop_image(self, image, window_adim):
        rows = image.shape[0]
        cols = image.shape[1]
        # print(rows, cols)

        # offset = self.offset
        # rows_count = int(rows/offset)
        # cols_count = int(cols/offset)

        # rows = rows_count*offset
        # cols = cols_count*offset

        # print(rows, cols)
        
        if window_adim is not None:
            x_min_px    = int(cols*window_adim[0])
            y_min_px    = int(rows*window_adim[1])
            x_max_px    = int(cols*window_adim[2])
            y_max_px    = int(rows*window_adim[3]) 

        else:
            x_min_px    = 0
            y_min_px    = 0
            x_max_px    = int(cols)
            y_max_px    = int(rows) 

        image = image[y_min_px:y_max_px, x_min_px:x_max_px]

        return image

    def convert_current_position_to_pixels(self, image, window_adim, cur_pos_center):
        # rospy.loginfo('Detecting obstacles started')

        if window_adim:
            image = self.crop_image(image, window_adim)

        rows = image.shape[0]
        cols = image.shape[1]

        cur_pos_center_indexes = None

        # Robot size - should be equal with the box size
        offset = self.offset
        
        rows_count = int(rows/offset)
        cols_count = int(cols/offset)

        rows = rows_count*offset
        cols = cols_count*offset
        
        for (i, row) in enumerate(range(0, rows, offset)):
            for (j, col) in enumerate(range(0, cols, offset)):
                if cur_pos_center is not None:
                    x = cur_pos_center[0]
                    y = cur_pos_center[1]
                    if (col <= x) and (x <= col+offset) and (row <= y) and (y <= row+offset+offset):
                        cur_pos_center_indexes = (j,i)
        return cur_pos_center_indexes

    def generate_map(self, image, window_adim, cur_pos_center, goal_pos_center):
        # rospy.loginfo('Detecting obstacles started')

        obstacles_map = []

        if window_adim:
            image = self.crop_image(image, window_adim)

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
                # obstacles_map.append(255 if is_obstacle else 0) 
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

        # rospy.loginfo('Detecting obstacles completed')
        
        return obstacles_map, cur_pos_center_indexes, goal_pos_center_indexes

    def does_image_contain_obstacles(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = self.hsv_min#np.array([0,0,245]) # b,g,r values
        upper = self.hsv_max#np.array([20,20,255]) # b,g,r values

        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)

        return True if 255 in output else False

        
if __name__=="__main__":

    rospy.init_node("obstacle_detector_node", log_level=rospy.DEBUG)
    cv_image = cv2.imread('/home/manos/Desktop/bright_obstacles.png') 


    # HSV limits for RED Haro
    hsv_min = (0, 100, 0)
    hsv_max = (5, 255, 255) 

    # We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
    window = [0.1, 0.07, 0.73, 0.91]

    obstacle_detector = ObstacleTracker(hsv_min, hsv_max)

    obstacles_map, rows, cols = obstacle_detector.generate_map(cv_image, window)

    start_index = (4,6)#50
    goal_index = (0,0)#4
    
    shortest_path = path_planning.find_shortest_path(obstacles_map, start_index, goal_index)
    # shortest_path = np.array(shortest_path).T
    # for pos in shortest_path:
    #     print(obstacle_detector.get_relative_position(cv_image, pos))
    
    while not rospy.is_shutdown():
        obstacle_detector.draw_map(cv_image, obstacles_map, window, shortest_path=shortest_path, start_index=start_index, goal_index=goal_index, imshow=True)

        #-- press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    

    rospy.logwarn("Shutting down")    
    cv2.destroyAllWindows()