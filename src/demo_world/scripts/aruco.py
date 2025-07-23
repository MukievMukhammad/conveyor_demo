#!/usr/bin/env python3

import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import numpy as np
import cv2
# import cv2.aruco as aruco

orig_img = None

def aruco_det(msg):
	global orig_img
	image = bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
	aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
	parameters = cv2.aruco.DetectorParameters()
	corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
		image, aruco_dict, parameters=parameters)
	# print(corners, ids, rejectedImgPoints)
	# print()
	cv2.aruco.drawDetectedMarkers(image, corners, ids)
	cv2.aruco.drawDetectedMarkers(image, rejectedImgPoints, borderColor=(100, 0, 240))

	mtx = np.array([
		[2490.86662,    0.     ,  677.11521],
        [    0.     , 2486.92174,  510.60805],
        [    0.     ,    0.     ,    1.     ]
	])
	dist = np.array([-0.053234, 0.343978, -0.002748, 0.000367, 0.000000])
	markerSizeInCM = 4
	orig_img = image
	rvec , tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corners, markerSizeInCM, mtx, dist)

	print(f"{tvec}")
	print("------------------------")

def showImages():
	global orig_img
	if orig_img is not None:
		cv2.imshow('Original', orig_img)
		cv2.waitKey(1)
	else: print(orig_img)

if __name__ == "__main__":
	rospy.init_node("conveyorCV", anonymous=True)
	bridge = cv_bridge.CvBridge()
	rospy.Subscriber("/pylon_camera_node/image_rect_color", Image, aruco_det)
	r = rospy.Rate(100)
	while not rospy.is_shutdown():
		showImages()
		r.sleep()
	# rospy.spin()
