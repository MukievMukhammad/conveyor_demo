#!/usr/bin/env python3

import sys
import rospy, cv2, cv_bridge, numpy, roslib
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist
from typing import Final
from conveyor.msg import Conveyor
from gazebo_msgs.msg import ModelStates
import math
import time
import numpy as np

import rospy, cv2, cv_bridge, numpy
from sensor_msgs.msg import Image
import geometry_msgs.msg
from typing import Final
from conveyor.msg import Conveyor

import rospy, cv2, cv_bridge



#!/usr/bin/env python
import sys, rospy, traceback, math, cv2, numpy as np, std_msgs.msg
from cv_bridge import CvBridge, CvBridgeError



class objectDetection():
    def __init__(self):
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/pylon_camera_node/image_raw', Image, self.callback)
        self.orig_img = None
        self.params = None

    def callback(self, data):
        l_h = cv2.getTrackbarPos("L - H", "Original")
        l_s = cv2.getTrackbarPos("L - S", "Original")
        l_v = cv2.getTrackbarPos("L - V", "Original")
        u_h = cv2.getTrackbarPos("U - H", "Original")
        u_s = cv2.getTrackbarPos("U - S", "Original")
        u_v = cv2.getTrackbarPos("U - V", "Original")
        try:
            cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
            hsv = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)
            lower_range = np.array([l_h, l_s, l_v])
            upper_range = np.array([u_h, u_s, u_v])
            mask = cv2.inRange(hsv, lower_range, upper_range)
            res = cv2.bitwise_and(cv_img, cv_img, mask=mask)
            mask_3 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            stacked = np.hstack((mask_3,cv_img,res))
        except CvBridgeError as exc:
            print(traceback.format_exc())
        if 'cv_img' in locals():
            self.orig_img = cv2.resize(stacked,None,fx=0.4,fy=0.4)

    def getTrackbarParams(self):
        return [cv2.getTrackbarPos('Param', 'Original')]


def showImages(obj):
    if obj.orig_img is not None:
        cv2.imshow('Original', obj.orig_img)
        # print("HEllo")
        # cv2.waitKey(10)

def handleTrackbarChanges(obj):
    params = obj.getTrackbarParams()

def trackbar_callback(x):
    pass

# A required callback method that goes into the trackbar function.
def nothing(x):
    pass

def createWindowsAndOriginal():     
    cv2.namedWindow('Original')

    # cv2.createTrackbar('Param', 'Original', 0, 179, trackbar_callback)
    cv2.createTrackbar("L - H", 'Original', 0, 179, nothing)
    cv2.createTrackbar("L - S", 'Original', 0, 255, nothing)
    cv2.createTrackbar("L - V", 'Original', 0, 255, nothing)
    cv2.createTrackbar("U - H", 'Original', 179, 179, nothing)
    cv2.createTrackbar("U - S", 'Original', 255, 255, nothing)
    cv2.createTrackbar("U - V", 'Original', 255, 255, nothing)
    # some more stuff

def main(args):
    createWindowsAndOriginal()
    od = objectDetection()
    rospy.init_node('objectdetection', anonymous=True)
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        # cv2.imshow('Parameter-Original', cv2.cvtColor(trackbar_img, cv2.COLOR_HSV2BGR))
        showImages(od)

        key = cv2.waitKey(1)
        if key == 27:
            break
        
        # handleTrackbarChanges(od)


        try:
            # rospy.spin()
            rate.sleep()
        except KeyboardInterrupt:
            print('Shutting down...')

    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)