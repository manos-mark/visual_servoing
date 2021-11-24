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

    # def blob_detect(self,
    #                 image,                  #-- The frame (cv standard)
    #                 hsv_min,                #-- minimum threshold of the hsv filter [h_min, s_min, v_min]
    #                 hsv_max,                #-- maximum threshold of the hsv filter [h_max, s_max, v_max]
    #                 blur=0,                 #-- blur value (default 0)
    #                 blob_params=None,       #-- blob parameters (default None)
    #                 search_window=None,     #-- window where to search as [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
    #                 imshow=False
    #             ):
    #     """
    #     Blob detecting function: returns keypoints and mask
    #     return keypoints, reversemask
    #     """

    #     #- Blur image to remove noise
    #     if blur > 0: 
    #         image    = cv2.blur(image, (blur, blur))
    #         #- Show result
    #         if imshow:
    #             cv2.imshow("Blur", image)
    #             cv2.waitKey(0)
            
    #     #- Search window
    #     if search_window is None: search_window = [0.0, 0.0, 1.0, 1.0]
        
    #     #- Convert image from BGR to HSV
    #     hsv     = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
    #     #- Apply HSV threshold
    #     mask    = cv2.inRange(hsv,hsv_min, hsv_max)
        
    #     #- Show HSV Mask
    #     if imshow:
    #         cv2.imshow("HSV Mask", mask)
        
    #     #- dilate makes the in range areas larger
    #     mask = cv2.dilate(mask, None, iterations=2)
    #     #- Show HSV Mask
    #     if imshow:
    #         cv2.imshow("Dilate Mask", mask)   
    #         cv2.waitKey(0)
            
    #     mask = cv2.erode(mask, None, iterations=2)
        
    #     #- Show dilate/erode mask
    #     if imshow:
    #         cv2.imshow("Erode Mask", mask)
    #         cv2.waitKey(0)
        
    #     #- Cut the image using the search mask
    #     mask = self.apply_search_window(mask, search_window)
        
    #     if imshow:
    #         cv2.imshow("Searching Mask", mask)
    #         cv2.waitKey(0)

    #     #- build default blob detection parameters, if none have been provided
    #     if blob_params is None:
    #         # Set up the SimpleBlobdetector with default parameters.
    #         params = cv2.SimpleBlobDetector_Params()
            
    #         # Change thresholds
    #         params.minThreshold = 0
    #         params.maxThreshold = 100
            
    #         # Filter by Area.
    #         params.filterByArea = True
    #         params.minArea = 30
    #         params.maxArea = 20000
            
    #         # Filter by Circularity
    #         params.filterByCircularity = False
    #         params.minCircularity = 0.1
            
    #         # Filter by Convexity
    #         params.filterByConvexity = False
    #         params.minConvexity = 0.5
            
    #         # Filter by Inertia
    #         params.filterByInertia =True
    #         params.minInertiaRatio = 0.5
            
    #     else:
    #         params = blob_params     

    #     #- Apply blob detection
    #     detector = cv2.SimpleBlobDetector_create(params)

    #     # Reverse the mask: blobs are black on white
    #     reversemask = 255-mask
        
    #     if imshow:
    #         cv2.imshow("Reverse Mask", reversemask)
    #         cv2.waitKey(0)
            
    #     keypoints = detector.detect(reversemask)

    #     return keypoints, reversemask


    # def draw_keypoints(self,
    #                 image,                   #-- Input image
    #                 keypoints,               #-- CV keypoints
    #                 line_color=(0,255,0),    #-- line's color (b,g,r)
    #                 imshow=False             #-- show the result
    #                 ):
    #     """
    #     Draw detected blobs: returns the image
    #     return(im_with_keypoints)
    #     """
        
    #     #-- Draw detected blobs as red circles.
    #     #-- cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS ensures the size of the circle corresponds to the size of blob
    #     # im_with_keypoints = cv2.drawKeypoints(image, keypoints, np.array([]), line_color, cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)
    #     x, y, size = keypoints
    #     im_with_keypoints = cv2.rectangle(image, (x-size,y-size),(x+size,y+size), (0,255,0), 1)

    #     if imshow:
    #         # Show keypoints
    #         cv2.imshow("Keypoints", im_with_keypoints)
            
    #     return(im_with_keypoints)


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

    
    # def draw_frame(self,
    #             image,
    #             dimension=0.3,      #- dimension relative to frame size
    #             line=2,              #- line's thickness
    #             imshow=True
    #     ):
    #     """
    #     Draw X Y frame
    #     return : image
    #     """
        
    #     image = image.copy()
    #     rows = image.shape[0]
    #     cols = image.shape[1]
    #     size = min([rows, cols])
    #     center_x = int(cols/2.0)
    #     center_y = int(rows/2.0)
        
    #     line_length = int(size*dimension)

    #     #-- X
    #     image = cv2.line(image, (center_x, center_y), (center_x+line_length, center_y), (0,0,255), line)
    #     #-- Y
    #     image = cv2.line(image, (center_x, center_y), (center_x, center_y+line_length), (0,255,0), line)
        
    #     return (image)

    
    # def apply_search_window(self, image, window_adim=[0.0, 0.0, 1.0, 1.0]):
    #     """
    #     Apply search window
    #     return: image
    #     """
    #     rows = image.shape[0]
    #     cols = image.shape[1]
    #     x_min_px    = int(cols*window_adim[0])
    #     y_min_px    = int(rows*window_adim[1])
    #     x_max_px    = int(cols*window_adim[2])
    #     y_max_px    = int(rows*window_adim[3])    
        
    #     #--- Initialize the mask as a black image
    #     mask = np.zeros(image.shape,np.uint8)
        
    #     #--- Copy the pixels from the original image corresponding to the window
    #     mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   
        
    #     #--- return the mask
    #     return(mask)
        
    
    # def blur_outside(self, image, blur=5, window_adim=[0.0, 0.0, 1.0, 1.0]):
    #     """
    #     Apply a blur to the outside search region
    #     """
    #     rows = image.shape[0]
    #     cols = image.shape[1]
    #     x_min_px    = int(cols*window_adim[0])
    #     y_min_px    = int(rows*window_adim[1])
    #     x_max_px    = int(cols*window_adim[2])
    #     y_max_px    = int(rows*window_adim[3])    
        
    #     # Initialize the mask as a black image
    #     mask    = cv2.blur(image, (blur, blur))
        
    #     # Copy the pixels from the original image corresponding to the window
    #     mask[y_min_px:y_max_px,x_min_px:x_max_px] = image[y_min_px:y_max_px,x_min_px:x_max_px]   

    #     return(mask)
        

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


    # def detect_blobs(self, image, window, obstacles_map):
    #     # Detect blobs
    #     keypoints, _ = self.blob_detect(image, self.hsv_min, self.hsv_max, blur=3, 
    #                                 blob_params=None, search_window=window, imshow=False)


    #     for keypoint in keypoints:
    #         x , y, size = self.get_blob_absolute_position(image, keypoint)

    #         # x , y, size = blob_detector_object.get_blob_relative_position(cv_image, keypoint)
    #         # blob_detector_object.publish_blob(x,y,size)
            
    #         # Draw Detection
    #         self.draw_keypoints(image, (x,y,size), imshow=False)
            

    #     # Draw window where we make detections
    #     image = self.draw_window(image, window, imshow=False)
    #     # blob_detector_object.draw_frame(cv_image)
        
    #     self.draw_map(image, obstacles_map, window, imshow=True)
        
    #     # blob_detector_object.draw_keypoints(cv_image, keypoints, imshow=True)

        
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
        # Get most recent Image
        # cv_image = mira_sensors_obj.get_image()
        # obstacle_detector.detect_blobs(cv_image, window, obstacles_map)

        obstacle_detector.draw_map(cv_image, obstacles_map, window, imshow=True)
        #-- press q to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    

    rospy.logwarn("Shutting down")    
    cv2.destroyAllWindows()