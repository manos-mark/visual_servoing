import cv2
import yaml
import numpy as np


def convert_center_to_corners(frame, center, offset, imshow=False):
	offset /= 2
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
	if corners is None:
		return None

	corners = corners.reshape((4, 2)) 
	(topLeft, topRight, bottomRight, bottomLeft) = corners
	# convert each of the (x, y)-coordinate pairs to integers
	topRight = (int(topRight[0]), int(topRight[1]))
	bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
	bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
	topLeft = (int(topLeft[0]), int(topLeft[1]))

	cX = int((topLeft[0] + bottomRight[0]) / 2.0)
	cY = int((topLeft[1] + bottomRight[1]) / 2.0)
	return (cX, cY)

def get_calibration_data(calibration_file):
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

ARUCO_DICT = {
	"DICT_4X4_50": cv2.aruco.DICT_4X4_50,
	"DICT_4X4_100": cv2.aruco.DICT_4X4_100,
	"DICT_4X4_250": cv2.aruco.DICT_4X4_250,
	"DICT_4X4_1000": cv2.aruco.DICT_4X4_1000,
	"DICT_5X5_50": cv2.aruco.DICT_5X5_50,
	"DICT_5X5_100": cv2.aruco.DICT_5X5_100,
	"DICT_5X5_250": cv2.aruco.DICT_5X5_250,
	"DICT_5X5_1000": cv2.aruco.DICT_5X5_1000,
	"DICT_6X6_50": cv2.aruco.DICT_6X6_50,
	"DICT_6X6_100": cv2.aruco.DICT_6X6_100,
	"DICT_6X6_250": cv2.aruco.DICT_6X6_250,
	"DICT_6X6_1000": cv2.aruco.DICT_6X6_1000,
	"DICT_7X7_50": cv2.aruco.DICT_7X7_50,
	"DICT_7X7_100": cv2.aruco.DICT_7X7_100,
	"DICT_7X7_250": cv2.aruco.DICT_7X7_250,
	"DICT_7X7_1000": cv2.aruco.DICT_7X7_1000,
	"DICT_ARUCO_ORIGINAL": cv2.aruco.DICT_ARUCO_ORIGINAL,
	"DICT_APRILTAG_16h5": cv2.aruco.DICT_APRILTAG_16h5,
	"DICT_APRILTAG_25h9": cv2.aruco.DICT_APRILTAG_25h9,
	"DICT_APRILTAG_36h10": cv2.aruco.DICT_APRILTAG_36h10,
	"DICT_APRILTAG_36h11": cv2.aruco.DICT_APRILTAG_36h11
}

def aruco_display(corners, ids, rejected, image):
	if len(corners) > 0:
		# flatten the ArUco IDs list
		ids = ids.flatten()
		# loop over the detected ArUCo corners
		for (markerCorner, markerID) in zip(corners, ids):
			# extract the marker corners (which are always returned in
			# top-left, top-right, bottom-right, and bottom-left order)
			corners = markerCorner.reshape((4, 2))
			(topLeft, topRight, bottomRight, bottomLeft) = corners
			# convert each of the (x, y)-coordinate pairs to integers
			topRight = (int(topRight[0]), int(topRight[1]))
			bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
			bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
			topLeft = (int(topLeft[0]), int(topLeft[1]))

			cv2.line(image, topLeft, topRight, (0, 255, 0), 2)
			cv2.line(image, topRight, bottomRight, (0, 255, 0), 2)
			cv2.line(image, bottomRight, bottomLeft, (0, 255, 0), 2)
			cv2.line(image, bottomLeft, topLeft, (0, 255, 0), 2)
			# compute and draw the center (x, y)-coordinates of the ArUco
			# marker
			cX = int((topLeft[0] + bottomRight[0]) / 2.0)
			cY = int((topLeft[1] + bottomRight[1]) / 2.0)
			cv2.circle(image, (cX, cY), 4, (0, 0, 255), -1)
			# draw the ArUco marker ID on the image
			cv2.putText(image, str(markerID),(topLeft[0], topLeft[1] - 10), cv2.FONT_HERSHEY_SIMPLEX,
				0.5, (0, 255, 0), 2)
			print("[Inference] ArUco marker ID: {}".format(markerID))
			# show the output image
	return image


