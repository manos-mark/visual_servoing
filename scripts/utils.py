import cv2
import yaml
import numpy as np


def convert_center_to_corners(frame, center, offset=15, imshow=False):
	"""Convert center to corners

	:param frame: Input image
	:type frame: ndarray
	:param center: Center point 
	:type center: tuple
	:param offset: Size of the robot in centimeters, defaults to 15
	:type offset: int, optional
	:param imshow: Show the image?, defaults to False
	:type imshow: bool, optional
	:return: Corners points
	:rtype: ndarray
	"""
	top_left = np.array([(center[0]-offset), (center[1]-offset)], dtype=np.int)
	bt_left = np.array([(center[0]-offset), (center[1]+offset)], dtype=np.int)
	top_right = np.array([(center[0]+offset), (center[1]-offset)], dtype=np.int)
	bt_right = np.array([(center[0]+offset), (center[1]+offset)], dtype=np.int)
	corners = np.array([[bt_left, top_left, top_right, bt_right]], dtype=np.float)
	
	if imshow:
		cv2.circle(frame, center, 6, (0, 140, 255), -1)	
		cv2.circle(frame, (int(top_left[0]), int(top_left[1])), 6, (0, 140, 255), -1)
		cv2.circle(frame, (int(bt_left[0]), int(bt_left[1])), 6, (0, 140, 255), -1)
		cv2.circle(frame, (int(top_right[0]), int(top_right[1])), 6, (0, 140, 255), -1)
		cv2.circle(frame, (int(bt_right[0]), int(bt_right[1])), 6, (0, 140, 255), -1)
		cv2.imshow('axis', frame)
  
	return corners


def convert_corners_to_center(corners):
	"""Convert corners to center

	:param corners: Corners points
	:type corners: ndarray
	:return: Center point
	:rtype: tuple
	"""
	if corners is None:
		return None

	corners = corners.reshape((4, 2)) 
	(top_left, top_right, bt_right, bt_left) = corners
	
 	# convert each of the (x, y)-coordinate pairs to integers
	top_left = (int(top_left[0]), int(top_left[1]))
	bt_right = (int(bt_right[0]), int(bt_right[1]))

	x = int((top_left[0] + bt_right[0]) / 2.0)
	y = int((top_left[1] + bt_right[1]) / 2.0)
	return (x, y)


def get_calibration_data(calibration_file):
	"""Get as input the path of the file that contains the calibration data
	and read them 

	:param calibration_file: Calibration file's path
	:type calibration_file: string
	:return: Height
	:rtype: float
 	:return: Width
	:rtype: float
	:return: Calibration matrix
	:rtype: ndarrat
 	:return: Distortion coefficients 
	:rtype: ndarray
	"""
	with open(calibration_file, "r") as data:
		try:
			calibration_data = yaml.safe_load(data)
		except yaml.YAMLError as exc:
			print(exc)
	camera_matrix = calibration_data["camera_matrix"]
	distortion_coef = calibration_data["distortion_coefficients"]
	h = calibration_data["image_width"]
	w = calibration_data["image_height"]
	cm = np.array(camera_matrix["data"]).reshape((camera_matrix["rows"],camera_matrix["cols"]))
	ds = np.array(distortion_coef["data"]).reshape((distortion_coef["rows"],distortion_coef["cols"]))

	return h,w,cm,ds
