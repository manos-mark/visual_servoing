#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
# from mira_sensors import MiraSensors
from geometry_msgs.msg import Point


class ObstacleTracker(object):

    def __init__(self, hsv_min, hsv_max):
        self.hsv_min = hsv_min
        self.hsv_max = hsv_max

    def draw_map(self,
                    image,              #- Input image
                    obstacles_map,
                    window_adim,        #- window in adimensional units
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

        i = 0
        for row in range(y_min_px+offset, y_max_px, offset):
            for col in range(x_min_px, x_max_px-offset, offset):
                if obstacles_map:
                    is_obstacle = obstacles_map[i]
                    text = 'X' if is_obstacle else 'O' 
                    color = (255,255,0) if is_obstacle else (255,0,0)
                    image = cv2.putText(image, text, (row,col), cv2.FONT_HERSHEY_SIMPLEX, 
                        1, color, 1, cv2.LINE_AA)
                i += 1

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

    def get_blob_relative_position(self, image, keyPoint):
        """
        Obtain the camera relative frame coordinate of one single keypoint
        return(x,y)
        """
        rows = float(image.shape[0])
        cols = float(image.shape[1])
        
        center_x    = 0.5*cols
        center_y    = 0.5*rows

        x = (keyPoint.pt[0] - center_x)/(center_x)
        y = (keyPoint.pt[1] - center_y)/(center_y)
        
        return x,y

    def get_blob_absolute_position(self, image, keyPoint):
        """
        Obtain the camera relative frame coordinate of one single keypoint
        return(x,y)
        """
        x = int(keyPoint.pt[0])
        y = int(keyPoint.pt[1])
        size = int((keyPoint.size/2))
        
        return x, y, size
    
    # def publish_blob(self, x, y ,size):
    #     blob_point = Point()
    #     blob_point.x = x
    #     blob_point.y = y
    #     blob_point.z = size 

    #     self.pub_blob.publish(blob_point)
    
    def generate_map(self, image, window_adim):
        rows = image.shape[0]
        cols = image.shape[1]

        obstacles_map = []
        
        x_min_px    = int(cols*window_adim[0])
        y_min_px    = int(rows*window_adim[1])
        x_max_px    = int(cols*window_adim[2])
        y_max_px    = int(rows*window_adim[3])  
        
        window_image = image[y_min_px:y_max_px, x_min_px:x_max_px]

        rows = window_image.shape[0]
        cols = window_image.shape[1]
        offset = 50
        
        for row in range(0, rows-offset, offset):
            for col in range(0, cols-offset, offset):
                box = window_image[col:col+offset, row:row+offset]
                is_obstacle = self.does_image_contain_obstacles(box)
                obstacles_map.append(is_obstacle) 
                
        # cv2.imshow('box r:'+str(row)+' c:'+str(col), box)
        return obstacles_map

    def does_image_contain_obstacles(self, image):
        image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        lower = self.hsv_min#np.array([0,0,245]) # b,g,r values
        upper = self.hsv_max#np.array([20,20,255]) # b,g,r values

        mask = cv2.inRange(image, lower, upper)
        output = cv2.bitwise_and(image, image, mask = mask)

        return True if 255 in output else False

        
if __name__=="__main__":

    rospy.init_node("blob_detector_node", log_level=rospy.DEBUG)
    cv_image = cv2.imread('/home/manos/Desktop/bright_obstacles.png') # mira_sensors_obj.get_image()


    # HSV limits for RED Haro
    hsv_min = (0, 100, 0)
    hsv_max = (5, 255, 255) 

    # We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
    window = [0.1, 0.07, 0.73, 0.91]

    obstacle_detector = ObstacleTracker(hsv_min, hsv_max)

    obstacles_map = obstacle_detector.generate_map(cv_image, window)
    while not rospy.is_shutdown():
        obstacle_detector.draw_map(cv_image, obstacles_map, window, imshow=True)
        #-- press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    

    rospy.logwarn("Shutting down")    
    cv2.destroyAllWindows()