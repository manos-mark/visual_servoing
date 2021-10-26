#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from your built-in webcam.
#
# Author:
# - Addison Sears-Collins
# - https://automaticaddison.com
 
# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from utils import ARUCO_DICT, aruco_display

 
def callback(data):
 
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  # current_frame = br.imgmsg_to_cv2(data)
  current_frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.height, data.width, -1)

  # load the ArUCo dictionary, grab the ArUCo parameters, and detect
  # the markers
  arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_ARUCO_ORIGINAL)
  arucoParams = cv2.aruco.DetectorParameters_create()
  corners, ids, rejected = cv2.aruco.detectMarkers(current_frame, arucoDict, parameters=arucoParams)

  detected_markers = aruco_display(corners, ids, rejected, current_frame)
  cv2.imshow("Image", detected_markers)

  cv2.waitKey(1)
      
def receive_message():
 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  rospy.init_node('video_sub_py', anonymous=True)
   
  # Node is subscribing to the video_frames topic
  rospy.Subscriber('/t265/stereo_ir/left/fisheye_image_raw', Image, callback)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__':
  # verify that the supplied ArUCo tag exists and is supported by
  # OpenCV
  receive_message()
