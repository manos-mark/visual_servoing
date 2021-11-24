#!/usr/bin/env python3
# Description:
# - Subscribes to real-time streaming video from simuation camera.

# Import the necessary libraries
import rospy # Python library for ROS
from sensor_msgs.msg import Image # Image is the message type
from visual_servoing.msg import Pose_estimation_vectors
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images
import cv2 # OpenCV library
import numpy as np
from utils import ARUCO_DICT, aruco_display, get_calibration_data
from detect_obstacles import ObstacleTracker
import path_planning


# Calibration Data
CALIBRATION_FILE_PATH = rospy.get_param('calibration_file')
HEIGHT, WIDTH, CAMERA_MATRIX, DISTORTION_COEF = get_calibration_data(CALIBRATION_FILE_PATH)



def pose_esitmation(frame, aruco_dict_type, matrix_coefficients, distortion_coefficients):
    '''
    frame - Frame from the video stream
    matrix_coefficients - Intrinsic matrix of the calibrated camera
    distortion_coefficients - Distortion coefficients associated with your camera

    return:-
    frame - The frame with the axis drawn on it
    '''

    frame = frame.copy()
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    cv2.aruco_dict = cv2.aruco.Dictionary_get(aruco_dict_type)
    parameters = cv2.aruco.DetectorParameters_create()
    message = Pose_estimation_vectors()

    corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict,parameters=parameters,
        cameraMatrix=matrix_coefficients,
        distCoeff=distortion_coefficients)
    #  The output of the ids is [[0],[1]] (tw0 markers)
    # If markers are detected
    if len(corners) > 0:
      for index, id in enumerate(ids):
        rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
          corners[index], 
          0.15, 
          matrix_coefficients,
          distortion_coefficients
        )
        # Draw a square around the markers
        cv2.aruco.drawDetectedMarkers(frame, corners) 

          # Draw Axis
        cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)  
        # print('Rotational vector:', rvec[0][0], 'Translational vector:', tvec[0][0])
        
        message.rotational.x, message.rotational.y, message.rotational.z  = rvec[0][0][0], rvec[0][0][1], rvec[0][0][2]
        message.translational.x, message.translational.y, message.translational.z = tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]
        
        # if the id[0] is the current position
        if id[0] == 0: 
          current_pub.publish(message)
        elif id[0] == 1:
          target_pub.publish(message)

            
    return frame

def on_image_received(data):
  # Used to convert between ROS and OpenCV images
  br = CvBridge()
 
  # Output debugging information to the terminal
  # rospy.loginfo("receiving video frame")
   
  # Convert ROS Image message to OpenCV image
  try:
    current_frame = br.imgmsg_to_cv2(data, 'bgr8')
  except:
    current_frame = np.frombuffer(data.data, dtype=np.uint8).reshape(data.shape[0], data.shape[1], -1)

  # current_frame = cv2.cvtColor(current_frame, cv2.COLOR_BGR2RGB)
  
  h,  w = current_frame.shape[:2]
  newcameramtx, roi = cv2.getOptimalNewCameraMatrix(CAMERA_MATRIX, DISTORTION_COEF, (WIDTH,HEIGHT), 1, (WIDTH,HEIGHT))
  
  # undistort
  dst = cv2.undistort(current_frame, CAMERA_MATRIX, DISTORTION_COEF, None, newcameramtx)
  
  # crop the image
  x, y, w, h = roi
  undistort_frame = dst[y:y+h, x:x+w]
  
  # load the ArUCo dictionary, grab the ArUCo parameters, and detect the markers
  # aruco_dict_type = cv2.aruco.DICT_4X4_100
  aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL #cv2.aruco['DICT_4X4_100']

  output = pose_esitmation(undistort_frame, aruco_dict_type, CAMERA_MATRIX, DISTORTION_COEF)

  cv2.imshow('Estimated Pose', output)

  obstacles_map, rows, cols = obstacle_detector.generate_map(undistort_frame, window)

  # costmap as 1-D array representation
  costmap = tuple(obstacles_map)
  # number of columns in the occupancy grid
  width = rows
  # number of rows in the occupancy grid
  height = cols
  start_index = 32
  goal_index = 5
  # side of each grid map square in meters
  resolution = 0.2

  shortest_path = path_planning.dijkstra(start_index, goal_index, rows, cols, costmap, resolution)
  print(shortest_path)

  # obstacle_detector.draw_map(output, obstacles_map, window, shortest_path, imshow=True)

  cv2.waitKey(1)
      
def receive_image():   
  # Node is subscribing to the video_frames topic
  camera = rospy.get_param('camera')
  rospy.Subscriber(camera, Image, on_image_received)
 
  # spin() simply keeps python from exiting until this node is stopped
  rospy.spin()
 
  # Close down the video stream when done
  cv2.destroyAllWindows()
  
if __name__ == '__main__': 
  # Tells rospy the name of the node.
  # Anonymous = True makes sure the node has a unique name. Random
  # numbers are added to the end of the name. 
  # node_name = rospy.get_param('estimate_pose_node')
  rospy.init_node('estimate_pose', anonymous=True)

  # topic_name = rospy.get_param('object_position')
  # print(topic_name)
  current_pub = rospy.Publisher('current_position', Pose_estimation_vectors, queue_size=10)
  target_pub = rospy.Publisher('target_position', Pose_estimation_vectors, queue_size=10)

  # HSV limits for RED Haro
  hsv_min = (0, 100, 0)
  hsv_max = (5, 255, 255)  
  
  obstacle_detector = ObstacleTracker(hsv_min, hsv_max)
  
  # We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
  window = [0.13, 0.05, 0.96, 0.80]

  receive_image()

  # data = cv2.imread('/home/manos/Desktop/obstacles.png')
  # while not rospy.is_shutdown():
  #   on_image_received(data)
  #   #-- press q to quit
  #   if cv2.waitKey(1) & 0xFF == ord('q'):
  #       break

  
