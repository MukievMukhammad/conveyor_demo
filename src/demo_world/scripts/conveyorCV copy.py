#!/usr/bin/env python3

import rospy, cv2, cv_bridge
from sensor_msgs.msg import Image
import geometry_msgs.msg
from typing import Final
from conveyor.msg import Conveyor
from collobaorative_assembly.srv import TakeObjFromConveyor
import numpy as np
from collections import deque


class ConveyorCV:
	image = None

	def __init__(self):
		self.WINDOW_ORIG: Final[str] = "original image"
		self.WINDOW_BIN: Final[str] = "binary"

		self.speed_pub = rospy.Publisher('/conveyor/speed', Conveyor, queue_size=10)
		rospy.wait_for_service('conveyor/take')
		self.take_from_cnvyr_srv = rospy.ServiceProxy('conveyor/take', TakeObjFromConveyor)

		self.mtx = np.array([
			[2490.86662,    0.     ,  677.11521],
			[    0.     , 2486.92174,  510.60805],
			[    0.     ,    0.     ,    1.     ]
		])
		self.dist = np.array([-0.053234, 0.343978, -0.002748, 0.000367, 0.000000])
		self.markerSizeInCM = 4
		self.base_frame = [0.021514432545543048, -0.22226675350695183, 0.5499995462155063]

		self.bridge = cv_bridge.CvBridge()
		self.image_buffer = deque(maxlen=100)


	def aruco_det(self, image):
		aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_5X5_1000)
		parameters = cv2.aruco.DetectorParameters()
		corners, ids, rejectedImgPoints = cv2.aruco.detectMarkers(
			image, aruco_dict, parameters=parameters)

		cv2.aruco.drawDetectedMarkers(image, corners, ids)
		try:
			rvec0 , tvec0, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 4, self.mtx, self.dist)
			tvec0 = tvec0[ids[0]]
			cv2.drawFrameAxes(image, self.mtx, self.dist, rvec0[ids[0]], tvec0, 1, 2)
			rvec1 , tvec1, _ = cv2.aruco.estimatePoseSingleMarkers(corners, 5.5, self.mtx, self.dist)
			tvec1 = tvec1[ids[1]]
			cv2.drawFrameAxes(image, self.mtx, self.dist, rvec1[ids[1]], tvec1, 1, 2)
		except:
			return False, False

		return [tvec1, tvec0], ids


	def camera_callback(self, msg):
		try:
			cv_image = cv_bridge.imgmsg_to_cv2(msg, "bgr8")
			self.image_buffer.append((msg.header.stamp, cv_image))
		except cv_bridge.CvBridgeError:
			pass
	

	def chot(self, msg):
		image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		orig_img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
		
		img_hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
		aruco_coords, aruco_ids = self.aruco_det(image)

		if aruco_coords == False:
			ConveyorCV.image = image
			return
		
		# TODO: extract to def finding cnvry area, guess it should has the max area
		# TODO: inside prev return find details and process: stop/run cnvry, call take detail
		# TODO: rewtire tf with tf2
		# TODO: add dynamic hsv adjusting with trackbars

		hue_min = 75
		saturation_min = 110
		value_min = 20

		hue_max = 179
		saturation_max = 255
		value_max = 90

		lower_yellow = np.array([hue_min,  saturation_min,  value_min])
		upper_yellow = np.array([hue_max, saturation_max, value_max])
		mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)


		contours, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		for cnt in contours:
			rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
			box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
			box = np.intp(box) # округление координат
			area = int(rect[1][0]*rect[1][1]) # вычисление площади
			if area > 300000:
				cv2.drawContours(image,[box],0,(0,255,0),2)
				conveyor_area_img = self.crop_area(orig_img, box)
				details_box = self.find_detail_on_cnvry(conveyor_area_img)
				# draw details bound
				for d_box in details_box:
					W = rect[1][0]
					H = rect[1][1]

					d_box[:,0] += min(box[:,0]) + int(W * 0.2 / 4)
					d_box[:,1] += min(box[:,1]) + int(H * 0.2)
					cv2.drawContours(image,[d_box],0,(0,255,0),2)
					cv2.circle(image, (int(np.mean(d_box[:,0])), int(np.mean(d_box[:,1]))), 5, (255, 0, 255), -1)

				# if next cube less than 500px from cnvry end than stop
				if np.any([box[:,1] < 2000 for box in details_box]):
					cnvyr = Conveyor()
					cnvyr.speed = 0
					self.speed_pub.publish(cnvyr)

					# take detail from cnvyr
					# --- start tf
					d_coords = np.array([[ int(np.mean(d_box[:,0])), int(np.mean(d_box[:,1])) ] for d_box in details_box])
					next_detail_coorns = (np.array([details_box[np.argmin(d_coords[:,1])]], dtype=np.float32), )[0]
					min_ditail_coord_pxl = d_coords[np.argmin(d_coords[:,1])]
					min_ditail_coord_pxl_cntr = [min_ditail_coord_pxl[0] - 1280/2, min_ditail_coord_pxl[1] - 1024/2]
					next_detail_coord = [i * 0.05 for i in min_ditail_coord_pxl_cntr] # in cm
					aruco0_coord = aruco_coords[1][0][0]
					# print(aruco0_coord)
					# print(next_detail_coord)
					detail_coord_aruco0 = [aruco0_coord[0] - next_detail_coord[0], aruco0_coord[1] - next_detail_coord[1], aruco0_coord[2]]
					# detail_coord_aruco0 = [aruco0_coord[0], aruco0_coord[1], aruco0_coord[2]]
					aruco1_coord = aruco_coords[0][0][0]
					detail_coord_aruco1 = [aruco1_coord[0] - aruco0_coord[0] + detail_coord_aruco0[0], aruco1_coord[1] -  aruco0_coord[1] + detail_coord_aruco0[1], aruco1_coord[2] - detail_coord_aruco0[2]]
					# from aruco1 to kuka base
					[0.021514432545543048, -0.22226675350695183, 0.5499995462155063]
					detail_coord_kuka_base = [-0.22226675350695183 - (detail_coord_aruco1[0]/100) - 0.02, 0.021514432545543048 - (detail_coord_aruco1[1]/100) - 0.005, 0.3017289625723313]
					take_coord = geometry_msgs.msg.Pose()
					take_coord.position.x = detail_coord_kuka_base[1] 
					take_coord.position.y = detail_coord_kuka_base[0]
					# --- end tf
					try:
						resp = self.take_from_cnvyr_srv(take_coord)
						print("resp: ", resp)
					except:
						print("Cannot call conveyor take srv: ")
				else:
					cnvyr = Conveyor()
					cnvyr.speed = 100
					self.speed_pub.publish(cnvyr)
			
		ConveyorCV.image = image
		# img_orig = cv2.resize(self.image, (1280, 1024))
		# mask = cv2.resize(mask, (1280, 1024))
		# cv2.imshow(self.WINDOW_ORIG, img_orig)
		# cv2.imshow(self.WINDOW_BIN, mask)
		# cv2.waitKey(10)


	def find_detail_on_cnvry(self, img):
		img_hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

		hue_min = 0
		saturation_min = 0
		value_min = 0

		hue_max = 179
		saturation_max = 255
		value_max = 125

		lower_yellow = np.array([hue_min,  saturation_min,  value_min])
		upper_yellow = np.array([hue_max, saturation_max, value_max])
		mask = cv2.inRange(img_hsv, lower_yellow, upper_yellow)
		mask = cv2.bitwise_not(mask) 

		contours, _ = cv2.findContours(mask.copy(), cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
		details_box = []
		for cnt in contours:	
			rect = cv2.minAreaRect(cnt) # пытаемся вписать прямоугольник
			box = cv2.boxPoints(rect) # поиск четырех вершин прямоугольника
			box = np.intp(box) # округление координат
			area = int(rect[1][0]*rect[1][1]) # вычисление площади
			if area > 100:
				details_box.append(box)
    
		# cv2.imshow("cnvy_deatil", mask)
		# cv2.waitKey(10)

		return details_box


	def crop_area(self, img_box, box):
		mult = 0.815

		Xs = [i[0] for i in box]
		Ys = [i[1] for i in box]
		x1 = min(Xs)
		x2 = max(Xs)
		y1 = min(Ys)
		y2 = max(Ys)


		center = (int((x1+x2)/2), int((y1+y2)/2))
		size = (int(mult*(x2-x1)),int(mult*(y2-y1)))
		cropped = cv2.getRectSubPix(img_box, size, center) 

		# cv2.imshow("crp_img", cropped)
		# cv2.waitKey(10)

		return cropped



def createWindowsAndOriginal():     
    cv2.namedWindow('Original')


def main():
	createWindowsAndOriginal()
	rate = rospy.Rate(10)

	while not rospy.is_shutdown():
		if ConveyorCV.image is not None:
			cv2.imshow('Original', ConveyorCV.image)
		else:
			pass

		

		key = cv2.waitKey(1)
		if key == 27:
			break
		
		try:
			rate.sleep()
		except KeyboardInterrupt:
			print('Shutting down...')

	cv2.destroyAllWindows()


if __name__ == "__main__":
	rospy.init_node("conveyorCV", anonymous=True)
	conveyorCV = ConveyorCV()
	rospy.Subscriber("/pylon_camera_node/image_rect_color", Image, conveyorCV.camera_callback)
	main()
	# rospy.spin()
