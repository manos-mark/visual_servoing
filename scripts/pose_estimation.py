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
from utils import ARUCO_DICT, aruco_display, get_calibration_data, convert_corners_to_center, convert_center_to_corners
from obstacle_detection import ObstacleTracker
import path_planning


# Calibration Data
CALIBRATION_FILE_PATH = rospy.get_param('calibration_file')
HEIGHT, WIDTH, CAMERA_MATRIX, DISTORTION_COEF = get_calibration_data(CALIBRATION_FILE_PATH)

# Aruco Marker ID's
CURRENT_ID = 0
TARGET_ID = 1

# Robot specification
ROBOT_SIZE_IN_PIXELS = 50
ROBOT_SIZE_IN_CENTIMETERS = 0.15


def estimate_pose(frame, 
			aruco_dict_type=cv2.aruco.DICT_ARUCO_ORIGINAL, 
			matrix_coefficients=CAMERA_MATRIX, 
			distortion_coefficients=DISTORTION_COEF, 
			imshow=True):
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

	corners, ids, rejected_img_points = cv2.aruco.detectMarkers(gray, cv2.aruco_dict ,parameters=parameters,
			cameraMatrix=matrix_coefficients,
			distCoeff=distortion_coefficients)

	# The output of the ids is [[0],[1]] (two markers)
	# If markers are detected
	if len(corners) > 0:
		for index, marker_id in enumerate(ids):
			rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
				corners[index], 
				ROBOT_SIZE_IN_CENTIMETERS, 
				matrix_coefficients,
				distortion_coefficients
			)
			
			# Draw a square around the markers
			cv2.aruco.drawDetectedMarkers(frame, corners) 

				# Draw Axis
			cv2.aruco.drawAxis(frame, matrix_coefficients, distortion_coefficients, rvec, tvec, 0.1)  
			# print('Rotational vector:', rvec[0][0], 'Translational vector:', tvec[0][0])
			
			if (marker_id[0] == CURRENT_ID) and (len(corners) >= 1): 
				# get the center of the position by converting the four corners 
				current_position['center'] = convert_corners_to_center(corners[0])
				current_position['corners'] = corners[0]       
				current_position['rvec'] = rvec
				current_position['tvec'] = tvec

			elif (marker_id[0] == TARGET_ID) and (len(corners) > 1):
				# get the center of the position by converting the four corners
				target_position['center'] = convert_corners_to_center(corners[1])
				target_position['corners'] = corners[1] 
				target_position['rvec'] = rvec
				target_position['tvec'] = tvec 

	if imshow:
		cv2.imshow('Estimated Pose', frame)
		cv2.waitKey(1)

	return (frame), current_position, target_position


def preprocess_image(data):
	# Used to convert between ROS and OpenCV images
	br = CvBridge()
 
	# Output debugging information to the terminal
	# rospy.loginfo("receiving video frame")
	
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

	return undistort_frame


def generate_message(rvec, tvec):
	message = Pose_estimation_vectors()
	if (rvec is not None) and (tvec is not None):
		message.rotational.x, message.rotational.y, message.rotational.z  = rvec[0][0][0], rvec[0][0][1], rvec[0][0][2]
		message.translational.x, message.translational.y, message.translational.z = tvec[0][0][0], tvec[0][0][1], tvec[0][0][2]

	return message
	

def on_image_received(data): 
	# Convert ROS Image message to OpenCV image
	undistort_frame = preprocess_image(data)

	image_with_pose, current_position, target_position = estimate_pose(undistort_frame, imshow=True)

	if (current_position is not None) and (target_position is not None):	
		publish_position(current_position, None)

		shortest_path_center_pixels, shortest_path, cur_pos_center_indexes = detect_obstacles_and_find_path(undistort_frame, image_with_pose, window, current_position, target_position)
		
		navigate_robot(shortest_path_center_pixels, shortest_path, cur_pos_center_indexes)

	else:
		rospy.loginfo('Waiting for current and target position')


def publish_position(current_position, target_position):
	if current_position:
		message = generate_message(current_position['rvec'], current_position['tvec'])
		current_position_publisher.publish(message)

	if target_position:
		message = generate_message(target_position['rvec'], target_position['tvec'])
		target_position_publisher.publish(message)


def detect_obstacles_and_find_path(undistort_frame, image_with_pose, window, current_position, target_position):
	global is_detecting_obstacles_completed, is_shortest_path_generated, shortest_path, shortest_path_center_pixels, cur_pos_center_indexes, goal_pos_center_indexes, obstacles_map

	if not is_detecting_obstacles_completed:
		obstacles_map, cur_pos_center_indexes, goal_pos_center_indexes = obstacle_detector.generate_map(
			undistort_frame, window, current_position['center'], target_position['center'], update_obstacles=True)
		is_detecting_obstacles_completed = True

	_, cur_pos_center_indexes, goal_pos_center_indexes = obstacle_detector.generate_map(
			undistort_frame, window, current_position['center'], target_position['center'], update_obstacles=False)
	
	# print((shortest_path is None), (cur_pos_center_indexes is not None), (goal_pos_center_indexes is not None))
	if is_detecting_obstacles_completed and not is_shortest_path_generated:
		shortest_path = path_planning.find_shortest_path(obstacles_map, cur_pos_center_indexes, 
			goal_pos_center_indexes)

		# We don't need the first and the last because we have markers 
		shortest_path = shortest_path[:-1]
		shortest_path_center_pixels = obstacle_detector.convert_center_to_pixels(image_with_pose, window, shortest_path)

		is_shortest_path_generated = True


	if is_detecting_obstacles_completed and is_shortest_path_generated:
		obstacle_detector.draw_map(image_with_pose, obstacles_map, 
			window, shortest_path, cur_pos_center_indexes, goal_pos_center_indexes, imshow=True)

	
	return shortest_path_center_pixels, shortest_path, cur_pos_center_indexes


def navigate_robot(shortest_path_center_pixels, shortest_path, cur_pos_center_indexes):
	if (shortest_path is None) or (shortest_path_center_pixels is None):
		return

	for i, (center_pixels, center_indexes) in enumerate(zip(shortest_path_center_pixels, shortest_path)):
		if (cur_pos_center_indexes == center_indexes):
			if i+1 < len(shortest_path_center_pixels):
				corners = convert_center_to_corners(shortest_path_center_pixels[i+1], ROBOT_SIZE_IN_CENTIMETERS)
				
				rvec, tvec, markerPoints = cv2.aruco.estimatePoseSingleMarkers(
					corners, 
					ROBOT_SIZE_IN_CENTIMETERS, 
					CAMERA_MATRIX, 
					DISTORTION_COEF
				)
				rospy.loginfo(f'Robot position {cur_pos_center_indexes}, middlepoint position {center_indexes} Move to path index {i+1} out of {len(shortest_path)}')
				message = generate_message(rvec, tvec)
				target_position_publisher.publish(message)
			
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
	current_position_publisher = rospy.Publisher('current_position', Pose_estimation_vectors, queue_size=10)
	target_position_publisher = rospy.Publisher('target_position', Pose_estimation_vectors, queue_size=10)

	# HSV limits for RED Haro
	hsv_min = (0, 100, 0)
	hsv_max = (5, 255, 255)  
	
	obstacle_detector = ObstacleTracker(hsv_min, hsv_max, ROBOT_SIZE_IN_PIXELS)
	
	# We define the detection area [x_min, y_min, x_max, y_max] adimensional (0.0 to 1.0) starting from top left corner
	window = [0.13, 0.05, 0.96, 0.80]

	# load the ArUCo dictionary, grab the ArUCo parameters, and detect the markers
	# aruco_dict_type = cv2.aruco.DICT_4X4_100
	# aruco_dict_type = cv2.aruco.DICT_ARUCO_ORIGINAL #cv2.aruco['DICT_4X4_100']

	current_position = target_position = shortest_path_center_pixels = cur_pos_center_indexes = goal_pos_center_indexes = obstacles_map = None

	is_detecting_obstacles_completed = False
	is_shortest_path_generated = False

	shortest_path = None

	current_position = dict(corners=None, center=None, rvec=None, tvec=None) 
	target_position = dict(corners=None, center=None, rvec=None, tvec=None)

	# receive_image()

	data = cv2.imread('/home/manos/Desktop/obstacles.png')
	while not rospy.is_shutdown():
		on_image_received(data)
		#-- press q to quit
		if cv2.waitKey(1) & 0xFF == ord('q'):
			break

	
