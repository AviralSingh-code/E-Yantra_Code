#!/usr/bin/env python3


import cv2
import numpy as np
import sys
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage


cv_image = None
h_min = 0


def empty(a):
    pass



cv2.namedWindow("TrackBars")
cv2.resizeWindow("TrackBars",250,250)
cv2.createTrackbar("Hue Max","TrackBars",0,179,empty)
cv2.createTrackbar("Sat Min","TrackBars",0,255,empty)
cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
cv2.createTrackbar("Sat Max","TrackBars",0,255,empty)
cv2.createTrackbar("Val Min","TrackBars",0,255,empty)
cv2.createTrackbar("Val Max","TrackBars",0,255,empty)

def callback(data):         #this callback is for color detection
    global cv_image
    # try:
    #     bridge = CvBridge()
    #     frame = bridge.imgmsg_to_cv2(data, "bgr8")      #the ROS format image is converted to bgr8 format -----> that is the format that is used in opencv
    #     imgContour = frame.copy()               #this function is used to copy the frame image to the imgContour
    #     imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)         #this is used to convert the image to HSV image
    #     x, y = 0, 0
    #     for color in tomatoColor:       #this for loop goes through the tomatoColor list and gets the h_min, h_max, s_min, s_max, v_min, v_max value for the red colour
    #         lower = np.array(color[0:3])        #this is for forming the lower range of the HSV colour space ----> i.e. h_min, s_min, v_min
    #         upper = np.array(color[3:6])        #this is for forming the upper range of the HSV colour space ----> i.e. h_max, s_max, v_max
    #         mask = cv2.inRange(imgHSV, lower, upper)            #for masking the image 
    #         x, y = getContours(mask)   
    # except CvBridgeError as e:          #for handling the error
    #     print(e)

    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "passthrough")
        cv_image = frame
        # imgContour = frame.copy()               #this function is used to copy the frame image to the imgContour
        # imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)         #this is used to convert the image to HSV image
        # x, y = 0, 0
        # for color in tomatoColor:       #this for loop goes through the tomatoColor list and gets the h_min, h_max, s_min, s_max, v_min, v_max value for the red colour
        #     lower = np.array(color[0:3])        #this is for forming the lower range of the HSV colour space ----> i.e. h_min, s_min, v_min
        #     upper = np.array(color[3:6])        #this is for forming the upper range of the HSV colour space ----> i.e. h_max, s_max, v_max
        #     mask = cv2.inRange(imgHSV, lower, upper)            #for masking the image 
        #     x, y = getContours(mask)   
    except CvBridgeError as e:          #for handling the error
        print(e)

def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    global cv_image
    while not rospy.is_shutdown():
        # data = rospy.wait_for_message("/camera/color/image_raw2", Image)
        # data = rospy.wait_for_message("/camera/color/image_raw/compressed", CompressedImage)
        image_sub = rospy.Subscriber("/camera/color/image_raw/compressed",CompressedImage, callback, queue_size=100)
        # try:
        #     bridge = CvBridge()
        #     frame = bridge.imgmsg_to_cv2(data, "bgr8")
        #     frame = bridge.imgmsg_to_cv2(data, "passthrough")
        #     cv_image = frame


        #     '''uncoment to view the visual of detection'''
            
        # except CvBridgeError as e:
        #     print(e)
        frame = cv_image
        cv2.imshow("frame", frame)
        cv2.waitKey(1)

        imgHsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
        h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
        h_max = cv2.getTrackbarPos("Hue Max","TrackBars")
        s_min = cv2.getTrackbarPos("Sat Min","TrackBars")
        s_max = cv2.getTrackbarPos("Sat Max","TrackBars")
        v_min = cv2.getTrackbarPos("Val Min","TrackBars")
        v_max = cv2.getTrackbarPos("Val Max","TrackBars")


        lower = np.array([h_min,s_min,v_min])
        upper = np.array([h_max,s_max,v_max])

        mask = cv2.inRange(imgHsv,lower,upper)

        imgResult = cv2.bitwise_and(frame,frame,mask=mask)

        cv2.imshow("HSV",imgHsv)
        cv2.imshow("ImageResult",imgResult)
        cv2.imshow("Mask",mask)
        cv2.waitKey(1)

if __name__ == '__main__':
    main(sys.argv)

