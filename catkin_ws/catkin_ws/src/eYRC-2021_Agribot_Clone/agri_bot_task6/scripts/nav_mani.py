#!/usr/bin/env python3


import cv2                                          #for importing opencv 
import numpy as np                                  #useful in the array operations
import roslib                                       #roslib is an internal library to write tools
import sys                                          #module to handle runtime environment
import rospy                                        #for use of pytho with ROS
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image                   #importing the image type sensor message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, CvBridgeError       #for importing CvBridge used to convert the ROS format images to opencv format
import tf2_ros                              #imporing tf2
import geometry_msgs.msg                    #for geometry messages
import tf_conversions                       #importing tf_conversions
import moveit_commander                     #moveit commander for motion planning
import moveit_msgs.msg                      #moveit messages
import actionlib                            #for importing for providing standard interface e.g. for returning point cloud
import math  
import time
import copy
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list




######## COLOR THRESHOLDING AND FILTERING ########
'''
h_min ----> Min value of Hue
s_min ----> Min value of Saturation
v_min ----> Min value of Value
h_max ----> Max value of Hue
s_max ----> Max value of Saturation
v_max ----> Max value of Value
'''

tomatoColor = [[0, 37, 117, 0, 250, 255]]       #------> change this values

###################################################






### GLOBAL VARIABLE ####





regions = {
    'bleft': 0.0,
    'bright': 0.0,
}

bleft = 0.0
tripLaser = 0.0
orient = 10.0





imgContour = None                           #imgContour is declared of the type None and it will hold the final image
depth_image = np.empty((480,640),float)     #this is a 2D array that hold the depth value of every pixel of the frame
midX = 0                                #this is the middle value of the entire span of the contour in the x - axis
midY = 0                        #this is the middle value of the entire span of the contour in the y - axis
linear_vel = 0.0
angular_vel = 0.0

flagEndMotion = False

flagPose = True         # this flag is used to first set the arm to a particular pose before starting the detection of the tomatoes 
flag = False           #this is to start detection and motion of the bot once the arm is set to a particular pose

ArucoId = -1            #this variable is used to hold the current aruco id of the marker visible ---> it is initialized to an invalid id of -1

pose = [0.0] * 4        #initializing the pose variable

flagDetect = False         #this variable is to check if the tomato is visible in the frame of the camera

flagExecution = True       #this variable is used to check if the execution of the motion of the arm was successful or not

# Motion Stage to handle the stage of motion of the agribot

motionStage = [False] * 15          #boolean list with all values False is defined
motionStage[0] = True               #the first stage is set to true to start the motion

#######################









class Ur5Moveit(object):

    def __init__(self):
        super(Ur5Moveit, self).__init__()
        moveit_commander.roscpp_initialize(sys.argv)
        # rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)
        self._hand_group = moveit_commander.MoveGroupCommander("gripper")


        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',moveit_msgs.msg.DisplayTrajectory,queue_size=20)


        planning_frame = group.get_planning_frame()
        eef_link = group.get_end_effector_link()
        group_names = robot.get_group_names()


        self.box_name = ''
        self.robot = robot
        self.scene = scene
        self.group = group
        self.display_trajectory_publisher = display_trajectory_publisher
        self.planning_frame = planning_frame
        self.eef_link = eef_link
        self.group_names = group_names


    def plan_cartesian_path(self, scale, valueX, valueY, valueZ):
        group = self.group

        # print("Problem###############")
        waypoints = []

        wpose = group.get_current_pose().pose
        # wpose.position.z += scale * 0.3  # First move up (z)
        # wpose.position.y += -1*scale * 0.4  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose = group.get_current_pose().pose
        # print(" x-initial : ",wpose.position.x)
        # print(" x-final : ",valueX)
        # wpose.position.x = valueX - wpose.position.x
        # wpose.position.y = valueY - wpose.position.y
        # wpose.position.z = valueZ - wpose.position.z


        wpose.position.y = valueY
        wpose.position.y -= 0.4
        waypoints.append(copy.deepcopy(wpose))



        wpose.position.x = valueX
        wpose.position.x -= 0.005
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = valueZ
        wpose.position.z += 0.01
        waypoints.append(copy.deepcopy(wpose))

        

        
        # wpose.position.y -= (-0.14064413407138388 - 0.6398005701040195)   #--------> this is very important over here we are doing wpose.position.y = wpose.position.y - (wpose.position.y - valueY)
        


        # wpose.position.x -= (0.05084215747822728 - 0.09873480355403039)
        # wpose.position.x = valueX
        # wpose.position.y -= (-0.14064413407138388 - 0.6398005701040195) - 0.2
        # wpose.position.z -= (1.0967436083169257 - 0.9450374418876081) # and sideways (y)
        # wpose.position.z = valueZ
        # waypoints.append(copy.deepcopy(wpose))



        wpose.position.y += 0.15
        waypoints.append(copy.deepcopy(wpose))


        

        # wpose.position.x += scale * 0.2  # Second move forward/backwards in (x)
        # waypoints.append(copy.deepcopy(wpose))

        # wpose.position.y -= scale * 0.1  # Third move sideways (y)
        # waypoints.append(copy.deepcopy(wpose))


        (plan, fraction) = group.compute_cartesian_path(waypoints, 0.01, 0.0)  

        return plan, fraction   


    def display_trajectory(self, plan):
        robot = self.robot
        display_trajectory_publisher = self.display_trajectory_publisher
        display_trajectory = moveit_msgs.msg.DisplayTrajectory()
        display_trajectory.trajectory_start = robot.get_current_state()
        display_trajectory.trajectory.append(plan)
        display_trajectory_publisher.publish(display_trajectory)


    def execute_plan(self, plan):

        group = self.group

        group.execute(plan, wait=True)


    def arm_robot(self,state):                          #this function is for taking the arm to predefined pose 
        if state == 1:                                  #When the state of the arm is set to 1 then the arm goes to the Drop_Pose
            self.group.set_named_target("Drop_Pose")   #Drop_Pose has been designed to drop the picked up tomatoes effectively in the bucket
            plan3 = self.group.go()                    #For planning and execution of the Drop_Pose ---> go() function handles both the work
        elif state == 0:                                #State 0 is for the Plant_Perception pose ---> using this pose the TF of all the tomatoes of the plant is collected at time 0
            self.group.set_named_target("PP_Pose2")
            plan3 = self.group.go()
        elif state == 2:
            self.group.set_named_target("Plant_Perception")
            plan3 = self.group.go()


    def gripper_robot(self,state):                      #this function is used to control the gripper and it accepts the state variable for setting the state of the gripper
        if state == 1:                                  #state 1 means that the gripper will be in the closed state
            self._hand_group.set_named_target("close")  #this is to set the target to a pre-defined pose of the gripper as close
            plan2 = self._hand_group.go()               #this line plans and executes the instructions given by moveit
        elif state == 0:                                #state 0 means that the gripper will be in the open state
            self._hand_group.set_named_target("open")   #this sets the state to open
            plan2 = self._hand_group.go()




def callback_depth(data_depth):             #this callback is for accessing the depth image

    global depth_image                  #this is to update the value in the global depth_image variable

    try:
        bridgeDepth = CvBridge()                #for creating the bridge object
        depth_image = bridgeDepth.imgmsg_to_cv2(data_depth, "32FC1") #for converting the depth image into depth matrix if 32FC1 encoding  ----> each pixel value is stored as one channel floating point with single precision
    except CvBridgeError as e:          #for handling errors
        print(e)





#$$$$$$$$$$$$$$$$$$$THIS SECTION IS USED SO THAT WE CAN READ THE ARUCO ID OF THE MARKER VISIBLE$$$$$$$$$$$$

def callback_Aruco(data):

    global ArucoId

    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # load the dictionary that was used to generate the markers
        dictionary = cv2.aruco.Dictionary_get(cv2.aruco.DICT_7X7_1000)

        # initializing the detector parameters with default values
        parameters =  cv2.aruco.DetectorParameters_create()

        # detect the markers in the frame
        corners, ids, rejectedCandidates = cv2.aruco.detectMarkers(frame, dictionary, parameters=parameters)

        if len(corners) > 0:
            # Flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                ArucoId = str(markerID)
                rospy.loginfo(ArucoId+" Reached")
    except CvBridgeError as e:
        print(e)

#$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$$





def getContours(img):           #this function is used to draw contours in their appropriate places
    global imgContour  
    global midX                 
    global midY
    global depth_image
    global flagDetect           #this variable will be set to true if the tomato is visible in the frame of the camera
    global maxCounter
    depth = 0                   #initial value of depth is taken to be 0
    cx = 320.5                  #the mid point of the x-axis is 320.5
    cy = 240.5                  #the mid point of the y-axis is 240.5
    fx = 554.387                #the focal length for x is 554.387
    fy = 554.387                #the focal length for y is 554.387
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)       #this is for getting all the contours that can be formed on the image
    x, y, w, h = 0, 0, 0, 0     #x, y, w, h are for getting the bounding rectangle dimensions for the drawing the contour
    midX = 0            #mid of x is set to 0
    midY = 0            #mid of y is set to 0
    counter = 0  #this counter is used to label the child frame ------> that means it's value is used to give unique label to every contour and hence giving unique value to every tomato
    for cnt in contours:                #for loop is for iterating over all the contours that needs to be drawn
        area = cv2.contourArea(cnt)         #this function is used to return the area of every contour that can be drawn on the image ---> that means every red tomato visible currently
        if area > 700:                   #this is for filtering the noise ---> that means if the area is >600 then it means we are looking at something substantial and we need to draw contour on that and broadcast the TF for that
            flagDetect = True            #the flagDetect is set to true so that the bot is slowed down so that the depth value of the tomatoes can be accessed easily
            peri = cv2.arcLength(cnt, True)     #this is to get the perimeter of the contour

            approx = cv2.approxPolyDP(cnt, 0.02 * peri, True)       #0.02 is a factor ---> it can be adjusted for better detection

            x, y, w, h = cv2.boundingRect(approx)               #this returns and stores the value of x, y, w, h

            cv2.circle(imgContour, (x+(w // 2), y+(h // 2)), max(h // 2, w // 2), (255, 0, 0), 2)   #this is the circle drawing function that draws image on imgContour with center x+(w/2) and y+(h/2) the next parameter is to get the radius of the circle that is the max of the entire spread
            cv2.circle(imgContour, (x+(w // 2), y+(h // 2)), 1, (255, 0, 0), -1)    #the (255, 0, 0) ----> is in the BGR format so max of blue and 0 values of red and green makes the circle blue in colour
            midX = x + w // 2       #this is the mid of the circle
            midY = y + h // 2       #this is the mid of the circle


            if midX != 0 and midY != 0 and midX <= 640 and midY <= 480:     #as the frame of the output image is 640 x 480 pixels so this is to prevent accessing values that are out of bound by any case
                depth = depth_image[midY][midX]         #the depth value is registered for the point (midX, midY)

                X = depth*((midX-cx)/fx)            #conversion to the world coordinates
                Y = depth*((midY-cy)/fy)            #conversion to the world coordinates
                Z = depth                           #conversion to the world coordinates


                br = tf2_ros.TransformBroadcaster()     #setting up the TF2 broadcaster
                t = geometry_msgs.msg.TransformStamped()        #broadcasting is stamped for every object
                t.header.stamp = rospy.Time.now()       #the head stamp is the current time that we use this makes it unique
                t.header.frame_id = "camera_link2"          #as the camera on the arm has the camera_link2 so we are using that
                t.child_frame_id = "obj"+str(counter)           #this is the naming convention where the is given as obj + value of the counter -----> obj1, obj2 etc.

                cv2.putText(imgContour, t.child_frame_id,               #this function is used to put text on the imgContour, the text is the child_frame_id, at the point (midX, midY) 
                (midX, midY), cv2.FONT_HERSHEY_SIMPLEX,         #cv2.FONT_HERSHEY_SIMPLEX ----> is the font used to label the image
                0.5, (255, 0, ), 2)             #0.5 is the font scale, (255, 0, 0) is for giving blue colour and 2 is the thickness


                ''' In this section we are trying to get the depth of the tomato visible ---> by doing this we will ensure that we accuate the arm only when the depth of the visible tomato
                is upto a certain limit so that we are unnecessarily don't try to pick up tomatoes on the other side of the plant and in doing so hit the plant and damage the arm'''

                depth = depth_image[midY][midX-w//2]    #the depth value is re-adjusted so that we ensure that the depth value that we get is for the tomato and not the thing just around the tomato because we are not stopping to check the depth value but we are drifting with a small value
                rospy.loginfo(depth)
                if depth <= 0.80:      #over here the depth value that allows us to pick the tomatoes on the side where the bot is standing is 0.75
                    '''In this section we will only broadcast the TF when the tomato visible is in the depth range from where it can be picked'''    

                    t.transform.translation.x = Z          #this is for transforming the world coordinates to the camera frame that is on the arm
                    t.transform.translation.y = -X         #this is for transforming the world coordinates to the camera frame that is on the arm
                    t.transform.translation.z = -Y         #this is for transforming the world coordinates to the camera frame that is on the arm

                    q = tf_conversions.transformations.quaternion_from_euler(0, 0, 0)       #for conversion euler to quaternion
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    br.sendTransform(t)         #for broadcating the TF 

                counter = counter+1 #as this loop loops through the number of times equal to the number of unique contours that can be drawn then if the counter is incremented same number of times it will have unique value starting from 1 for every contour
                if counter > maxCounter:
                    maxCounter = counter
        else:
            flagDetect = False              #flagDetect is set to false if the tomato is not visible
            maxCounter = 0
    cv2.imshow("Result",imgContour)         #this is for displaying the final image with all the contours on it
    cv2.waitKey(1)          #this is for adding a 1 millisecond delay

    return x + w // 2, y + h // 2  # for mid of the box is returned using this










def callback(data):         #this callback is for color detection
    global imgContour     
    try:
        bridge = CvBridge()
        frame = bridge.imgmsg_to_cv2(data, "bgr8")      #the ROS format image is converted to bgr8 format -----> that is the format that is used in opencv
        imgContour = frame.copy()               #this function is used to copy the frame image to the imgContour
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)         #this is used to convert the image to HSV image
        x, y = 0, 0
        for color in tomatoColor:       #this for loop goes through the tomatoColor list and gets the h_min, h_max, s_min, s_max, v_min, v_max value for the red colour
            lower = np.array(color[0:3])        #this is for forming the lower range of the HSV colour space ----> i.e. h_min, s_min, v_min
            upper = np.array(color[3:6])        #this is for forming the upper range of the HSV colour space ----> i.e. h_max, s_max, v_max
            mask = cv2.inRange(imgHSV, lower, upper)            #for masking the image 
            x, y = getContours(mask)   
    except CvBridgeError as e:          #for handling the error
        print(e)




def forward():
    global linear_vel
    global angular_vel
    linear_vel = 0.2
    angular_vel = 0.0



def stopMotion():
    global linear_vel
    global angular_vel
    linear_vel = 0.0
    angular_vel = 0.0



def odom_callback(data):
    global orient
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x, y, z, w])[2]]
    # rospy.loginfo(pose[2])
    orient = pose[2]




def laser_callback(msg):
    global regions
    global bleft
    global tripLaser
    bleft = min(msg.ranges[680:720])                    #------------> change this
    tripLaser = min(msg.ranges[700:720])      #----> set this range also
    if bleft == 0:
        bleft = 50
    if tripLaser == 0:
        tripLaser = 50
    regions = {
        'bleft': min(bleft, 1.5),
        'tripLaser': min(tripLaser, 2.5),               #---------> adjust this value 2.5 ---> so that it is not triggered when it is turning with p controller but it gets triggered when it returns to the start position
    }




def Pcontroller():                          #-----> check the PController
    global angular_vel
    global linear_vel
    global regions
    kp = 4  # --> 1.5
    angular_vel = kp * (regions['bleft'] - 0.60)
    # edit
    if angular_vel > 0.8:
        angular_vel = 0.5
    # edit
    if (-0.3 > regions['bleft'] - 0.60) or (regions['bleft'] - 0.60 > 0.3):
        linear_vel = 0.1











def main(args):                                                                         #this is the main method

    rospy.init_node('object_detection_manipulation', anonymous=True)                                 #for initializing the node with name object_detection
    tfBuffer = tf2_ros.Buffer()                                                         #for setting the buffer
    listener = tf2_ros.TransformListener(tfBuffer)                                      #for using the TF listener
    ur5 = Ur5Moveit()                                                                   #for setting up the ur5 object of Ur5Moveit class
    depth_sub = rospy.Subscriber("/camera/depth/image_raw2", Image, callback_depth)     #for setting the depth image subscriber
    image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)           #for setting the color image subscriber
    image_sub_aruco = rospy.Subscriber("/ebot/camera1/image_raw", Image, callback_Aruco)
    rospy.Subscriber('/ebot/laser/scan',LaserScan, laser_callback)      #setting up the subscriber
    rospy.Subscriber('/odom', Odometry, odom_callback)  #setting up the subscriber
    pub = rospy.Publisher('/cmd_vel',Twist,queue_size=10)   #setting up the publisher
    
    ur5_pose_1 = geometry_msgs.msg.Pose()                                               #for using the pose 
    velocity_msg = Twist()

    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    pub.publish(velocity_msg)
    global linear_vel
    global angular_vel
    global flagPose
    global flag
    global ArucoId
    global flagDetect
    global flagExecution
    global maxCounter
    global flagEndMotion
    flagNewPose = False         #to get to a new pose to make the accuation of the arm from this new pose in case it fails the first time

    hold = ''
    
    flagStage0 = False
    flagIntermediate = False
    flagLastStage = False
    start_time = time.time()
    seconds = 4
    while not rospy.is_shutdown():                      #while loop
        current_time = time.time()
        elapsed_time = current_time - start_time
        if elapsed_time >= seconds:
            if flagEndMotion is True:           #gets triggered when the bot returns to its start location after completing the motion
                    stopMotion()                                    #to stop the motion when the allignment of the aruco 
                    velocity_msg.linear.x = linear_vel
                    velocity_msg.angular.z = angular_vel
                    pub.publish(velocity_msg)
                    rospy.loginfo('Mission Accomplished!')
                    break

            if flagPose is True:
                flagPose = False
                ur5.arm_robot(0)
                rospy.sleep(0.1)
                flag = True


            elif flag is True:

                for i in range(maxCounter+1):       #this loop actually checks which tomato is in the permissible depth range 
                    try:
                        trans = tfBuffer.lookup_transform('base_link', 'obj'+str(i), rospy.Time(0))      #for getting the TF of obj0 ----> tomato in the scene
                        hold = str(i)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue

                    if hold != '':
                        break


                
                if hold == '':  #if no tomato is visible the hold will be blank string so the bot will keep moving
                    
                    
                    if (0.5 < orient < 2.5) and (flagStage0 is False):
                        flagStage0 = True
                        while regions['bleft'] > 1.0:
                            rospy.loginfo(1)
                            forward()
                            velocity_msg.linear.x = linear_vel
                            velocity_msg.angular.z = angular_vel
                            pub.publish(velocity_msg)
                            rospy.sleep(0.1)

                        forward()
                        rospy.loginfo(6)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(2)

                    elif (-2.5 < orient < -0.5) and (regions['tripLaser'] >= 2.4) and (flagIntermediate is False):       #-----> check the tripLaser value
                        flagIntermediate = True
                        forward()
                        rospy.loginfo(10)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(4)

                        if orient > 0.15:
                            while orient > 0.15:
                                rospy.loginfo(8)
                                linear_vel = 0.0
                                angular_vel = -0.2
                                velocity_msg.linear.x = linear_vel
                                velocity_msg.angular.z = angular_vel
                                pub.publish(velocity_msg)
                                rospy.sleep(0.1)
                        elif orient < -0.15:
                            while orient < -0.15:
                                rospy.loginfo(9)
                                linear_vel = 0.0
                                angular_vel = 0.2
                                velocity_msg.linear.x = linear_vel
                                velocity_msg.angular.z = angular_vel
                                pub.publish(velocity_msg)
                                rospy.sleep(0.1)


                        forward()
                        rospy.loginfo(3)
                        velocity_msg.linear.x = linear_vel                #-----> check the linear velocity
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(12)                              #--------> check this

                    elif (-2.5 < orient < -0.5) and (regions['tripLaser'] >= 2.4) and (flagLastStage is False):
                        flagLastStage = True
                        forward()
                        rospy.loginfo(13)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(4)                              #-----> check this value 

                        stopMotion()
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(0.1)

                        flagEndMotion = True

                    forward()
                    Pcontroller()
                    rospy.loginfo(5)
                    if flagDetect is False:
                        velocity_msg.linear.x = linear_vel
                    elif flagDetect is True:
                        velocity_msg.linear.x = linear_vel - 0.1                    #-----------> check this value
                    velocity_msg.angular.z = angular_vel
                    pub.publish(velocity_msg)
                    rospy.sleep(0.1)

                    ArucoId = '-1' #set to invalid id -1

                    continue


                
                try:
                    transAruco = tfBuffer.lookup_transform('sjcam_link', 'aruco'+ArucoId, rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    velocity_msg.linear.x = 0.1             #till the time the TF of the aruco is not available then the bot is moving with a small velocity of 0.1
                    velocity_msg.angular.z = 0.0
                    pub.publish(velocity_msg)
                    continue
                while transAruco.transform.translation.y > 0.01 or transAruco.transform.translation.y < -0.01:  #when the TF is available but the center of the Aruco is not in the range 
                    if transAruco.transform.translation.y > 0.01:   #if the bot has moved forward by any chance then a negative velocity will bring it back
                        velocity_msg.linear.x = -0.1
                        velocity_msg.angular.z = 0.0
                        pub.publish(velocity_msg)
                    elif transAruco.transform.translation.y < -0.01:        #if the bot has moved backward then a positive velocity will do the work
                        velocity_msg.linear.x = 0.1
                        velocity_msg.angular.z = 0.0
                        pub.publish(velocity_msg)
                    transAruco = tfBuffer.lookup_transform('sjcam_link', 'aruco'+ArucoId, rospy.Time(0))




                stopMotion()                                    #to stop the motion when the allignment of the aruco 
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)

                trans = tfBuffer.lookup_transform('base_link', 'obj'+hold, rospy.Time(0))  #the TF of the tomato is read again when the bot is completely stopped so that the TF of the tomato is perfectly correct

                rospy.loginfo('obj'+hold+' Identified at '+ArucoId)







                tomato_Position_X_Transform = trans.transform.translation.x           #for storing the x value of transform for the tomato 2
                tomato_Position_Y_Transform = trans.transform.translation.y           #for storing the y value of transform for the tomato 2
                tomato_Position_Z_Transform = trans.transform.translation.z           #for storing the z value of transform for the tomato 2

                base_link_Position_X = 0.16                  #this is the x position of the base_link ----> our reference point
                base_link_Position_Y = 0                     #this is the y position of the base_link ----> our reference point
                base_link_Position_Z = 0.53 





                ur5_pose_1.position.x = base_link_Position_X + tomato_Position_X_Transform
                ur5_pose_1.position.y = base_link_Position_Y + tomato_Position_Y_Transform
                ur5_pose_1.position.z = base_link_Position_Z + tomato_Position_Z_Transform 

                cartesian_plan, fraction = ur5.plan_cartesian_path(1, ur5_pose_1.position.x, ur5_pose_1.position.y, ur5_pose_1.position.z)
                ur5.display_trajectory(cartesian_plan)
                ur5.execute_plan(cartesian_plan)


                ur5.gripper_robot(1)                #state 1 of the gripper is to close the gripper
                rospy.sleep(0.1)

                rospy.loginfo('obj'+hold+' Picked')

                ur5.arm_robot(1)                    #to bring the arm to the bucket
                rospy.sleep(0.1)
                ur5.gripper_robot(0)                #to open the gripper
                rospy.sleep(0.1)

                rospy.loginfo('obj'+hold+' Dropped in '+'Basket')


                flagPose = True                #to set this variable to true again so that the arm positions itself correctly after picking up the tomato
                flag = False               #this will prevent further execution till the arm has not positioned itself correctly
                hold = ''
                flagNewPose = False

    del ur5


if __name__ == '__main__':
    main(sys.argv) 

