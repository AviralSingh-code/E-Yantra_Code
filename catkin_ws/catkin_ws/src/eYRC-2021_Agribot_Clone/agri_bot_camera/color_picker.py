#!/usr/bin/env python3


import cv2
import numpy as np
# import roslib
import sys
import rospy
import sensor_msgs.msg
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
# import tf2_ros
# import geometry_msgs.msg
# import tf_conversions


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



    





# def callback(data):
#     # Initializing variables
#     global cv_image
#     global h_min
#     try:
#         bridge = CvBridge()
#         frame = bridge.imgmsg_to_cv2(data, "bgr8")
#         cv_image = frame
#         # frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

#         '''uncoment to view the visual of detection'''
#         cv2.imshow("frame", frame)
#         cv2.waitKey(1)
#     except CvBridgeError as e:
#         print(e)





    



def main(args):
    rospy.init_node('aruco_tf', anonymous=True)
    # subscribing to /ebot/camera1/image_raw topic which is the image frame of sjcam camera
    # image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)
    while not rospy.is_shutdown():
        # topic = rospy.get_param('/camera/color/image_raw2')
        data = rospy.wait_for_message("/camera/color/image_raw2", Image)
        try:
            bridge = CvBridge()
            frame = bridge.imgmsg_to_cv2(data, "bgr8")
            cv_image = frame
            # frame = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

            '''uncoment to view the visual of detection'''
            cv2.imshow("frame", frame)
            cv2.waitKey(1)
        except CvBridgeError as e:
            print(e)


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

        # try:
        #     rospy.spin()
        # except KeyboardInterrupt:
        #     print("Shutting down")
        # cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)

















































# # # #!/usr/bin/env python


# # from __future__ import print_function
# # import rospy
# # from sensor_msgs.msg import Image
# # from cv_bridge import CvBridge, CvBridgeError
# # import cv2
# # import numpy as np


# # def empty(a):
# #     pass


# # class colorPicker(object):
# #     def __init__(self):
# #         self.image_sub = rospy.Subscriber("/camera/color/image_raw2",Image, self.camera_callback)
# #         self.bridge_object = CvBridge()



# #     def camera_callback(self,data):
# #         try:
# #             cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
# #         except CvBridgeError as e:
# #             print(e)



# #         cv2.namedWindow("TrackBars")
# #         cv2.resizeWindow("TrackBars",250,250)


# #         cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
# #         cv2.createTrackbar("Hue Max","TrackBars",0,179,empty)
# #         cv2.createTrackbar("Sat Min","TrackBars",0,255,empty)
# #         cv2.createTrackbar("Sat Max","TrackBars",0,255,empty)
# #         cv2.createTrackbar("Val Min","TrackBars",0,255,empty)
# #         cv2.createTrackbar("Val Max","TrackBars",0,255,empty)


# #         imgHsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
# #         h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
# #         h_max = cv2.getTrackbarPos("Hue Max","TrackBars")
# #         s_min = cv2.getTrackbarPos("Sat Min","TrackBars")
# #         s_max = cv2.getTrackbarPos("Sat Max","TrackBars")
# #         v_min = cv2.getTrackbarPos("Val Min","TrackBars")
# #         v_max = cv2.getTrackbarPos("Val Max","TrackBars")



# #         lower = np.array([h_min,s_min,v_min])
# #         upper = np.array([h_max,s_max,v_max])


# #         mask = cv2.inRange(imgHsv,lower,upper)



# #         imgResult = cv2.bitwise_and(cv_image,cv_image,mask=mask)

# #         # cv2.imshow("HSV Image",imgHsv)

# #         # cv2.imshow("Mask",mask)

# #         cv2.imshow("Result",cv_image)

# #         cv2.waitKey(3)



# # def main():
# #     rospy.init_node('color_Picker_node',anonymous=True)
# #     color_Picker_object = colorPicker()
# #     try:
# #         rospy.spin()
# #     except KeyboardInterrupt:
# #         print("Shutting down")

# # if __name__=='__main__':
# #     main()






# # # # import rospy
# # # # import cv2

# # # # from sensor_msgs.msg import Image
# # # # from cv_bridge import CvBridge, CvBridgeError


# # # from __future__ import print_function
# # # import rospy
# # # from sensor_msgs.msg import Image
# # # from cv_bridge import CvBridge, CvBridgeError
# # # import cv2
# # # import numpy as np

# # # h_min = 0

# # # image = None

# # # def empty(a):
# # #     pass







# # # # def helper(image):
# # # #     global h_min
# # # #     cv2.imshow("Helper_Output",image)

    

    

# # # #     rospy.loginfo(h_min)




# # # class camera_1:


# # #     def __init__(self):
# # #         self.image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, self.callback)
    

# # #     def callback(self,data):
# # #         global image
# # #         bridge = CvBridge()

# # #         try:
# # #             cv_image = bridge.imgmsg_to_cv2(data, desired_encoding="bgr8")
# # #             # helper(cv_image)
# # #         except CvBridgeError as e:
# # #             rospy.logerr(e)





# # #         cv2.namedWindow("TrackBars")
# # #         cv2.resizeWindow("TrackBars",250,250)


# # #         cv2.createTrackbar("Hue Min","TrackBars",0,179,empty)
# # #         cv2.createTrackbar("Hue Max","TrackBars",0,179,empty)
# # #         cv2.createTrackbar("Sat Min","TrackBars",0,255,empty)
# # #         cv2.createTrackbar("Sat Max","TrackBars",0,255,empty)
# # #         cv2.createTrackbar("Val Min","TrackBars",0,255,empty)
# # #         cv2.createTrackbar("Val Max","TrackBars",0,255,empty)





# # #         imgHsv = cv2.cvtColor(cv_image,cv2.COLOR_BGR2HSV)
# # #         h_min = cv2.getTrackbarPos("Hue Min","TrackBars")
# # #         h_max = cv2.getTrackbarPos("Hue Max","TrackBars")
# # #         s_min = cv2.getTrackbarPos("Sat Min","TrackBars")
# # #         s_max = cv2.getTrackbarPos("Sat Max","TrackBars")
# # #         v_min = cv2.getTrackbarPos("Val Min","TrackBars")
# # #         v_max = cv2.getTrackbarPos("Val Max","TrackBars")
    
# # #         image = cv_image

# # #         # # resized_image = cv2.resize(image, (250, 250)) 

# # #         # # #cv2.imshow("Camera output normal", image)
# # #         # # cv2.imshow("Camera output resized", resized_image)
        



# # #         # lower = np.array([h_min,s_min,v_min])
# # #         # upper = np.array([h_max,s_max,v_max])


# # #         # mask = cv2.inRange(imgHsv,lower,upper)



# # #         # imgResult = cv2.bitwise_and(cv_image,cv_image,mask=mask)

# # #         # cv2.imshow("HSV Image",imgHsv)

# # #         # cv2.imshow("Mask",mask)

# # #         # cv2.imshow("Result",imgResult)

# # #         cv2.waitKey(5000)



# # # def main():
# # #   camera_1()
  
# # #   try:
# # #     rospy.spin()
# # #   except KeyboardInterrupt:
# # #     rospy.loginfo("Shutting down")
  
# # #   cv2.destroyAllWindows()

# # # if __name__ == '__main__':
# # #     rospy.init_node('camera_read', anonymous=False)
# # #     main()





































# # # from __future__ import print_function
# # # import cv2 as cv
# # # import argparse
# # # max_value = 255
# # # max_value_H = 360//2
# # # low_H = 0
# # # low_S = 0
# # # low_V = 0
# # # high_H = max_value_H
# # # high_S = max_value
# # # high_V = max_value
# # # window_capture_name = 'Video Capture'
# # # window_detection_name = 'Object Detection'
# # # low_H_name = 'Low H'
# # # low_S_name = 'Low S'
# # # low_V_name = 'Low V'
# # # high_H_name = 'High H'
# # # high_S_name = 'High S'
# # # high_V_name = 'High V'
# # # def on_low_H_thresh_trackbar(val):
# # #     global low_H
# # #     global high_H
# # #     low_H = val
# # #     low_H = min(high_H-1, low_H)
# # #     cv.setTrackbarPos(low_H_name, window_detection_name, low_H)
# # # def on_high_H_thresh_trackbar(val):
# # #     global low_H
# # #     global high_H
# # #     high_H = val
# # #     high_H = max(high_H, low_H+1)
# # #     cv.setTrackbarPos(high_H_name, window_detection_name, high_H)
# # # def on_low_S_thresh_trackbar(val):
# # #     global low_S
# # #     global high_S
# # #     low_S = val
# # #     low_S = min(high_S-1, low_S)
# # #     cv.setTrackbarPos(low_S_name, window_detection_name, low_S)
# # # def on_high_S_thresh_trackbar(val):
# # #     global low_S
# # #     global high_S
# # #     high_S = val
# # #     high_S = max(high_S, low_S+1)
# # #     cv.setTrackbarPos(high_S_name, window_detection_name, high_S)
# # # def on_low_V_thresh_trackbar(val):
# # #     global low_V
# # #     global high_V
# # #     low_V = val
# # #     low_V = min(high_V-1, low_V)
# # #     cv.setTrackbarPos(low_V_name, window_detection_name, low_V)
# # # def on_high_V_thresh_trackbar(val):
# # #     global low_V
# # #     global high_V
# # #     high_V = val
# # #     high_V = max(high_V, low_V+1)
# # #     cv.setTrackbarPos(high_V_name, window_detection_name, high_V)
# # # parser = argparse.ArgumentParser(description='Code for Thresholding Operations using inRange tutorial.')
# # # parser.add_argument('/camera/color/image_raw2', help='Camera divide number.', default=0, type=int)
# # # args = parser.parse_args()
# # # cap = cv.VideoCapture(args.camera)
# # # cv.namedWindow(window_capture_name)
# # # cv.namedWindow(window_detection_name)
# # # cv.createTrackbar(low_H_name, window_detection_name , low_H, max_value_H, on_low_H_thresh_trackbar)
# # # cv.createTrackbar(high_H_name, window_detection_name , high_H, max_value_H, on_high_H_thresh_trackbar)
# # # cv.createTrackbar(low_S_name, window_detection_name , low_S, max_value, on_low_S_thresh_trackbar)
# # # cv.createTrackbar(high_S_name, window_detection_name , high_S, max_value, on_high_S_thresh_trackbar)
# # # cv.createTrackbar(low_V_name, window_detection_name , low_V, max_value, on_low_V_thresh_trackbar)
# # # cv.createTrackbar(high_V_name, window_detection_name , high_V, max_value, on_high_V_thresh_trackbar)
# # # while True:
    
# # #     ret, frame = cap.read()
# # #     if frame is None:
# # #         break
# # #     frame_HSV = cv.cvtColor(frame, cv.COLOR_BGR2HSV)
# # #     frame_threshold = cv.inRange(frame_HSV, (low_H, low_S, low_V), (high_H, high_S, high_V))
    
    
# # #     cv.imshow(window_capture_name, frame)
# # #     cv.imshow(window_detection_name, frame_threshold)
    
# # #     key = cv.waitKey(30)
# # #     if key == ord('q') or key == 27:
# # #         break





# """Allows the user to calibrate for HSV filtering later."""

# # Standard libraries
# from argparse import ArgumentParser

# # ROS libraries
# import rospy
# from cv_bridge import CvBridge
# from sensor_msgs.msg import Image

# # Other libraries
# import cv2
# import numpy as np


# def nothing():
#     """Does nothing."""

#     pass


# def calibrator(msg, args):
#     """Updates the calibration window"""

#     bridge = args
#     raw = bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
#     hsv = cv2.cvtColor(raw, cv2.COLOR_BGR2HSV)

#     # get info from track bar and appy to result
#     h_low = cv2.getTrackbarPos('H_low', 'HSV Calibrator')
#     s_low = cv2.getTrackbarPos('S_low', 'HSV Calibrator')
#     v_low = cv2.getTrackbarPos('V_low', 'HSV Calibrator')
#     h_high = cv2.getTrackbarPos('H_high', 'HSV Calibrator')
#     s_high = cv2.getTrackbarPos('S_high', 'HSV Calibrator')
#     v_high = cv2.getTrackbarPos('V_high', 'HSV Calibrator')

#     # Normal masking algorithm
#     lower = np.array([h_low, s_low, v_low])
#     upper = np.array([h_high, s_high, v_high])

#     mask = cv2.inRange(hsv, lower, upper)

#     result = cv2.bitwise_and(raw, raw, mask=mask)

#     cv2.imshow('HSV Calibrator', result)
#     cv2.waitKey(30)


# def main(node, subscriber):
#     """Creates a camera calibration node and keeps it running."""

#     # Initialize node
#     rospy.init_node(node)

#     # Initialize CV Bridge
#     bridge = CvBridge()

#     # Create a named window to calibrate HSV values in
#     cv2.namedWindow('HSV Calibrator')

#     # Creating track bar
#     cv2.createTrackbar('H_low', 'HSV Calibrator', 0, 179, nothing)
#     cv2.createTrackbar('S_low', 'HSV Calibrator', 0, 255, nothing)
#     cv2.createTrackbar('V_low', 'HSV Calibrator', 0, 255, nothing)

#     cv2.createTrackbar('H_high', 'HSV Calibrator', 50, 179, nothing)
#     cv2.createTrackbar('S_high', 'HSV Calibrator', 100, 255, nothing)
#     cv2.createTrackbar('V_high', 'HSV Calibrator', 100, 255, nothing)

#     # Subscribe to the specified ROS topic and process it continuously
#     rospy.Subscriber(subscriber, Image, calibrator, callback_args=(bridge))

#     rospy.spin()


# if __name__ == "__main__":
#     PARSER = ArgumentParser()
#     PARSER.add_argument("--subscribe", "-s",
#                         default="/camera/color/image_raw2",
#                         help="ROS topic to subcribe to (str)", type=str)
#     PARSER.add_argument("--node", "-n", default="CameraCalibrator",
#                         help="Node name (str)", type=str)
#     ARGS = PARSER.parse_args()

#     main(ARGS.node, ARGS.subscribe)



# import sys, rospy, traceback, math, cv2, numpy as np, std_msgs.msg
# from sensor_msgs.msg import Image
# from cv_bridge import CvBridge, CvBridgeError


# class objectDetection():
#     def __init__(self):
#         self.bridge = CvBridge()
#         self.sub = rospy.Subscriber('/camera/color/image_raw2', Image, self.callback)
#         self.orig_img = None
#         self.params = None

#     def callback(self, data):
#         self.params = self.getTrackbarParams()
#         try:
#             cv_img = self.bridge.imgmsg_to_cv2(data, "bgr8")
#         except CvBridgeError as exc:
#             print(traceback.format_exc())
#         if 'cv_img' in locals():
#             self.orig_img = cv_img

#     def getTrackbarParams(self):
#         return [cv2.getTrackbarPos('Param', 'Parameter-Trackbars')]


# def showImages(obj):
#     if obj.orig_img is not None:
#         cv2.imshow('Original', obj.orig_img)

# def handleTrackbarChanges(obj):
#     params = obj.getTrackbarParams()

# def trackbar_callback(x):
#     pass

# def createWindowsAndTrackbars():     
#     cv2.namedWindow('Parameter-Trackbars')
#     cv2.namedWindow('Original')

#     cv2.createTrackbar('Param', 'Parameter-Trackbars', 0, 179, trackbar_callback)
#     # some more stuff

# def main(args):
#     createWindowsAndTrackbars()
#     od = objectDetection()
#     rospy.init_node('objectdetection', anonymous=True)

#     while (1):
#         [...]
#         cv2.imshow('Parameter-Trackbars', cv2.cvtColor(od.orig_img, cv2.COLOR_HSV2BGR))
#         showImages(od)

#         k = cv2.waitKey(1) & 0xFF
#         if k == 27:
#             break

#         handleTrackbarChanges(od)

#         try:
#             rospy.spin()
#         except KeyboardInterrupt:
#             print('Shutting down...')
#         cv2.destroyAllWindows()

# if __name__ == '__main__':
#     main(sys.argv)