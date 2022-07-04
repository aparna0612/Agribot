#!/usr/bin/env python3


import cv2
import numpy as np
import roslib
import sys
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import tf2_ros
import geometry_msgs.msg
import tf_conversions



tomatoColor = [[0, 37, 117, 0, 250, 255]]
imgContour = None
midX = 0
midY = 0

def getContours(img):
    global imgContour
    global midX
    global midY
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    x, y, w, h = 0, 0, 0, 0
    midX = 0
    midY = 0
    for cnt in contours:
        area = cv2.contourArea(cnt)
        # print(area)
        if area > 100:                          #set to 100 for proper filteration
            peri = cv2.arcLength(cnt, True)

            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)

            x, y, w, h = cv2.boundingRect(approx)

            cv2.circle(imgContour, (x+(w // 2), y+(h // 2)), max(h // 2, w // 2), (255, 0, 0), 1)
            cv2.circle(imgContour, (x+(w // 2), y+(h // 2)), 1, (255, 0, 0), -1)                #-1 at the end is to fill the circle
            midX = x + w // 2
            midY = y + h // 2

    return x + w // 2, y + h // 2  # for mid of the box



def callback(data):
    global imgContour
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        imgContour = frame.copy()
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        x, y = 0, 0
        for color in tomatoColor:
            lower = np.array(color[0:3])
            upper = np.array(color[3:6])
            mask = cv2.inRange(imgHSV, lower, upper)
            x, y = getContours(mask)   
        cv2.imshow("Result",imgContour)
        cv2.waitKey(1) 
    except CvBridgeError as e:
        print(e)




def callback_depth(data_depth):
    global midX
    global midY
    try:
        bridgeDepth = CvBridge()
        depth_image = bridgeDepth.imgmsg_to_cv2(data_depth, "32FC1")
        if midX != 0 and midY != 0 and midX <= 640 and midY <= 480:             #for proper thresholding
            rospy.loginfo(depth_image[midY][midX]) #maybe the coordinates are inverted
    except CvBridgeError as e:
        print(e)









def main(args):
    rospy.init_node('object_detection', anonymous=True)
    image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)  
    depth_sub = rospy.Subscriber("/camera/depth/image_raw2", Image, callback_depth) 
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)