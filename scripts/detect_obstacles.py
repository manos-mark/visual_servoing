#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
# from mira_sensors import MiraSensors
from geometry_msgs.msg import Point
import path_planning 


class ObstacleTracker(object):

    def __init__(self, hsv_min, hsv_max):
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max

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
        
        image = image.copy()
        rows = image.shape[0]
        cols = image.shape[1]
        
        x_min_px    = int(cols*window_adim[0])
        y_min_px    = int(rows*window_adim[1])
        x_max_px    = int(cols*window_adim[2])
        y_max_px    = int(rows*window_adim[3])  
        
        #-- Draw a rectangle from top left to bottom right corner
        image = cv2.rectangle(image,(x_min_px,y_min_px),(x_max_px,y_max_px),color,line)
        
        offset = 50
        
        for row in range(y_min_px, y_max_px, offset):
            image = cv2.line(image, (x_min_px, row), (x_max_px, row), (100,0,0), line)

        for col in range(x_min_px, x_max_px, offset):
            image = cv2.line(image, (col, y_min_px), (col, y_max_px), (100,0,0), line)

        for (i, col) in enumerate(range(y_min_px+offset, y_max_px, offset)):
            for (j, row) in enumerate(range(x_min_px, x_max_px-offset, offset)):
                
                if obstacles_map is not None:
                    is_obstacle = obstacles_map[j][i]
                    text = 'X' if is_obstacle else 'O' 
                    color = (255,255,0) if is_obstacle else (255,0,0)
                    center = (int(row+offset/4), int(col-offset/4))
                    
                    image = cv2.putText(image, text, center, cv2.FONT_HERSHEY_SIMPLEX, 
                        1, color, 3, cv2.LINE_AA)

                    pos = self.get_relative_position(image, (row,col))
                    pos = '(%0.1f,%0.1f)' % (pos[0]*10, pos[1]*10)
                    
                    image = cv2.putText(image, pos, (row, col), cv2.FONT_HERSHEY_SIMPLEX, 
                        0.3, color, 1, cv2.LINE_AA)

                    if start_index is not None:
                        if (j,i) == start_index:
                                color = (255, 255, 255)
                                cv2.putText(image, 'S', center, cv2.FONT_HERSHEY_SIMPLEX, 
                                    1, color, 5, cv2.LINE_AA)
                            
                    if goal_index is not None:
                        if (j,i) == goal_index:
                            color = (255, 255, 255)
                            cv2.putText(image, 'G', center, cv2.FONT_HERSHEY_SIMPLEX, 
                                1, color, 5, cv2.LINE_AA)
                
                #Radius of circle
                radius = 6
                # Orange color in BGR
                color = (0, 140, 255)
                # Line thickness of 2 px
                thickness = -1

                if shortest_path is not None:
                    if (j,i) in shortest_path:
                        cv2.circle(image, center, radius, color, thickness)


        # i = 0
        # for col in range(y_min_px+offset, y_max_px, offset):
        #     for row in range(x_min_px, x_max_px-offset, offset):

        #         if start_index is not None:
        #             if i == start_index:
        #                     color = (255, 255, 255)
        #                     cv2.putText(image, 'S', (row,col), cv2.FONT_HERSHEY_SIMPLEX, 
        #                         1, color, 5, cv2.LINE_AA)
                        
        #         if goal_index is not None:
        #             if i == goal_index:
        #                 color = (0, 255, 255)
        #                 cv2.putText(image, 'G', (row,col), cv2.FONT_HERSHEY_SIMPLEX, 
        #                     1, color, 5, cv2.LINE_AA)
            
        #         #Radius of circle
        #         radius = 6
        #         # Orange color in BGR
        #         color = (0, 140, 255)
        #         # Line thickness of 2 px
        #         thickness = -1

        #         if shortest_path is not None:
        #             if i in shortest_path:
        #                 center_coordinates = (row,col)
        #                 cv2.circle(image, center_coordinates, radius, color, thickness)
        #         i += 1


        if imshow:
            # Show keypoints
            cv2.imshow("Map", image)

        return(image)

    def draw_window(self,
                    image,              #- Input image
                    window_adim,        #- window in adimensional units
                    color=(255,0,0),    #- line's color
                    line=5,             #- line's thickness
                    imshow=False        #- show the image
                ):
        """
        Draw search window: returns the image
        return(image)
        """
        
        rows = image.shape[0]
        cols = image.shape[1]
        
        x_min_px    = int(cols*window_adim[0])
        y_min_px    = int(rows*window_adim[1])
        x_max_px    = int(cols*window_adim[2])
        y_max_px    = int(rows*window_adim[3])  
        
        #-- Draw a rectangle from top left to bottom right corner
        image = cv2.rectangle(image,(x_min_px,y_min_px),(x_max_px,y_max_px),color,line)
        
        if imshow:
            # Show keypoints
            cv2.imshow("Keypoints", image)

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
    
    # def publish_obstacle(self, x, y ,size):
    #     blob_point = Point()
    #     blob_point.x = x
    #     blob_point.y = y
    #     blob_point.z = size 

    #     self.pub_blob.publish(blob_point)
    
    def generate_map(self, image, window_adim, cur_pos_center, goal_pos_center):
        rows = image.shape[0]
        cols = image.shape[1]

        obstacles_map = []
        obstacles_map_coordinates = []
        
        x_min_px    = int(cols*window_adim[0])
        y_min_px    = int(rows*window_adim[1])
        x_max_px    = int(cols*window_adim[2])
        y_max_px    = int(rows*window_adim[3]) 
        
        window_image = image[y_min_px:y_max_px, x_min_px:x_max_px]

        rows = window_image.shape[0]
        cols = window_image.shape[1]

        cur_pos_center_indexes = goal_pos_center_indexes = None

        # Robot size - should be equal with the box size
        offset = 50
        
        for (i, row) in enumerate(range(0, rows-offset, offset)):
            row_list = []

            for (j, col) in enumerate(range(0, cols-offset, offset)):
                box = window_image[col:col+offset, row:row+offset]
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
            
            # cv2.imshow('box r:'+str(row)+' c:'+str(col), box)
        
        # obstacles_map = np.array(obstacles_map).T
        # obstacles_map = obstacles_map.flatten().squeeze()
        # for ob in obstacles_map:
        #     print(ob)
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