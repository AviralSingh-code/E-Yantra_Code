#!/usr/bin/env python3


import cv2  # for importing opencv
import numpy as np  # useful in the array operations
import roslib  # roslib is an internal library to write tools
import sys  # module to handle runtime environment
import rospy  # for use of pytho with ROS
from geometry_msgs.msg import Twist, Pose
from sensor_msgs.msg import Image  # importing the image type sensor message
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from cv_bridge import CvBridge, \
    CvBridgeError  # for importing CvBridge used to convert the ROS format images to opencv format
import tf2_ros  # imporing tf2
import geometry_msgs.msg  # for geometry messages
import tf_conversions  # importing tf_conversions
import moveit_commander  # moveit commander for motion planning
import moveit_msgs.msg  # moveit messages
import actionlib  # for importing for providing standard interface e.g. for returning point cloud
import math  # for importing the math library
import copy
import time
from sensor_msgs.msg import Image, CompressedImage
from std_msgs.msg import Bool
from ur_msgs.srv import SetIO
from std_msgs.msg import Float32MultiArray

######## COLOR THRESHOLDING AND FILTERING ########
'''
h_min ----> Min value of Hue
s_min ----> Min value of Saturation
v_min ----> Min value of Value
h_max ----> Max value of Hue
s_max ----> Max value of Saturation
v_max ----> Max value of Value
'''

tomatoColor = [[29, 146, 150, 179, 255, 255], [0, 120, 3, 9, 255, 186]]  # Defining the HSV color space of red tomatoes with format [h_min, s_min, v_min, h_max, s_max, v_max]
# tomatoColor = [[0, 37, 117, 0, 250, 255]], [0, 120, 3, 9, 255, 186]
# [29, 146, 150, 179, 255, 255]
###################################################


### GLOBAL VARIABLE ####

imgContour = None  # imgContour is declared of the type None and it will hold the final image
depth_image = np.empty((480, 640), float)  # this is a 2D array that hold the depth value of every pixel of the frame
midX = 0  # this is the middle value of the entire span of the contour in the x - axis
midY = 0  # this is the middle value of the entire span of the contour in the y - axis
linear_vel = 0.0
angular_vel = 0.0

maxCounter = 0

bleft = 0.0
tripLaser = 0.0
orient = 10.0
init_orient = 10.0
init_flag = False

regions = {  # this is to hold the laser values of paricular regions
    'bleft': 0.0,
    'tripLaser': 0.0
}

flagPose = True  # this flag is used to first set the arm to a particular pose before starting the detection of the tomatoes
flag = False  # this is to start detection and motion of the bot once the arm is set to a particular pose

flagEndMotion = False
flag_inp = True
flagCheck = False

ArucoId = -1  # this variable is used to hold the current aruco id of the marker visible ---> it is initialized to an invalid id of -1

pose = [0.0] * 4  # initializing the pose variable

flagDetect = False  # this variable is to check if the tomato is visible in the frame of the camera

flagExecution = True  # this variable is used to check if the execution of the motion of the arm was successful or not


#######################

'''
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

        display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                       moveit_msgs.msg.DisplayTrajectory, queue_size=20)

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

        wpose.position.y = valueY
        wpose.position.y -= 0.4
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.x = valueX
        # wpose.position.x -= 0.75
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.z = valueZ
        wpose.position.z += 0.01
        waypoints.append(copy.deepcopy(wpose))

        wpose.position.y += 0.15
        waypoints.append(copy.deepcopy(wpose))

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

    # def open_gripper(self):
    #     rospy.wait_for_service('/ur_hardware_interface/set_io')
    #     # open gripper
    #     spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    #     spawn_srv(fun=1, pin=16, state=0.0)
    #     rospy.sleep(0.5)
    #
    # # Function to close the gripper
    # def close_gripper(self):
    #     rospy.wait_for_service('/ur_hardware_interface/set_io')
    #     # close gripper
    #     spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    #     spawn_srv(fun=1, pin=16, state=1.0)
    #     rospy.sleep(0.5)


    def arm_robot(self, state):  # this function is for taking the arm to predefined pose
        if state == 1:  # When the state of the arm is set to 1 then the arm goes to the Drop_Pose
            self.group.set_named_target(
                "Drop_Pose")  # Drop_Pose has been designed to drop the picked up tomatoes effectively in the bucket
            plan3 = self.group.go()  # For planning and execution of the Drop_Pose ---> go() function handles both the work
        elif state == 2:
            self.group.set_named_target("Plant_Perception")
            plan3 = self.group.go()

'''



def open_gripper():
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    # open gripper
    spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    spawn_srv(fun=1, pin=16, state=0.0)
    rospy.sleep(0.5)


def close_gripper():
    rospy.wait_for_service('/ur_hardware_interface/set_io')
    # close gripper
    spawn_srv = rospy.ServiceProxy('/ur_hardware_interface/set_io', SetIO)
    spawn_srv(fun=1, pin=16, state=1.0)
    rospy.sleep(0.5)


def set_joint(state):
    pub_joint_pose_wait_ack = rospy.Publisher("/set_joint_value_target_wait_topic", Float32MultiArray, queue_size=10)
    rospy.sleep(2)

    array_temp = Float32MultiArray()
    # array_temp.data = [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint]
    if state == 1:
        # array_temp.data = [1.5708, -0.1736, -0.8157, 1.0761, 0.0000, 0.0000]
        array_temp.data = [-0.069, -2.321, -1.675, -3.682, -1.658, 4.712]
    elif state == 2:
        array_temp.data = [0.0000, 0.5901, 0.0521, 0.0000, 0.0000, 0.0000]
    elif state == 3:
        array_temp.data = [1.5708, -0.1736, -0.8157, -0.3817, 0.0000, 0.0000]

    pub_joint_pose_wait_ack.publish(array_temp)

def set_pose(x, y, z, q1, q2, q3, q4):
    pub_ee_pose_wait_ack = rospy.Publisher("/set_ee_pose_target_wait_topic", Pose, queue_size=10)
    rospy.sleep(2)

    ee_pose = Pose()
    ee_pose.position.x = x
    ee_pose.position.y = y
    ee_pose.position.z = z
    ee_pose.orientation.x = q1
    ee_pose.orientation.y = q2
    ee_pose.orientation.z = q3
    ee_pose.orientation.w = q4

    pub_ee_pose_wait_ack.publish(ee_pose)

# def set_pose():
#     pub_ee_pose_wait_ack = rospy.Publisher("/set_ee_pose_target_wait_topic", Pose, queue_size=10)
#     rospy.sleep(2)
#
#     ee_pose = Pose()
#     ee_pose.position.x = 0.2572230126965329
#     ee_pose.position.y = 0.1919011468409372
#     ee_pose.position.z = 0.8920275541055108
#
#     ee_pose.orientation.x = 0.5473376689156836
#     ee_pose.orientation.y = 0.43518671591792285
#     ee_pose.orientation.z = 0.5518910850218367
#     ee_pose.orientation.w = 0.4543679442345587
#
#     pub_ee_pose_wait_ack.publish(ee_pose)





def image_cb(data):
    global imgContour
    np_arr = np.frombuffer(data.data, np.uint8)
    rgb_frame = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    frame = rgb_frame
    imgContour = rgb_frame.copy()
    try:
        imgHSV = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)  # this is used to convert the image to HSV image
        x, y = 0, 0
        for color in tomatoColor:  # this for loop goes through the tomatoColor list and gets the h_min, h_max, s_min, s_max, v_min, v_max value for the red colour
            lower = np.array(
                color[0:3])  # this is for forming the lower range of the HSV colour space ----> i.e. h_min, s_min, v_min
            upper = np.array(
                color[3:6])  # this is for forming the upper range of the HSV colour space ----> i.e. h_max, s_max, v_max
            mask = cv2.inRange(imgHSV, lower, upper)  # for masking the image
            x, y = getContours(mask)

    except CvBridgeError as e:  # for handling the error
        print(e)




def image_dp(data):
    global depth_image
    bridge = CvBridge()
    depth_frame = bridge.imgmsg_to_cv2(data, "passthrough")
    depth_image = depth_frame


def callback_depth(data_depth):             #this callback is for accessing the depth image

    global depth_image                  #this is to update the value in the global depth_image variable

    try:
        bridgeDepth = CvBridge()                #for creating the bridge object
        depth_image = bridgeDepth.imgmsg_to_cv2(data_depth, "32FC1") #for converting the depth image into depth matrix if 32FC1 encoding  ----> each pixel value is stored as one channel floating point with single precision
    except CvBridgeError as e:          #for handling errors
        print(e)

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

def getContours(img):  # this function is used to draw contours in their appropriate places
    global imgContour
    global midX
    global midY
    global depth_image
    global flagDetect  # this variable will be set to true if the tomato is visible in the frame of the camera
    global maxCounter
    global flag_inp

    #     [fx  0 cx]
    # K = [ 0 fy cy]
    #     [ 0  0  1]

    depth = 0.0  # initial value of depth is taken to be 0
    cx = 321.8162536621094  # the mid point of the x-axis is 320.5
    cy = 241.31869506835938  # the mid point of the y-axis is 240.5
    fx = 610.2002563476562  # the focal length for x is 554.387
    fy = 609.3546752929688  # the focal length for y is 554.387
    contours, hierarchy = cv2.findContours(img, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_NONE)  # this is for getting all the contours that can be formed on the image
    x, y, w, h = 0, 0, 0, 0  # x, y, w, h are for getting the bounding rectangle dimensions for the drawing the contour
    midX = 0  # mid of x is set to 0
    midY = 0  # mid of y is set to 0
    counter = 0  # this counter is used to label the child frame ------> that means it's value is used to give unique label to every contour and hence giving unique value to every tomato
    for cnt in contours:  # for loop is for iterating over all the contours that needs to be drawn
        area = cv2.contourArea(
            cnt)  # this function is used to return the area of every contour that can be drawn on the image ---> that means every red tomato visible currently
        if area > 50:  # this is for filtering the noise ---> that means if the area is >800 then it means we are looking at something substantial and we need to draw contour on that and broadcast the TF for that
            flagDetect = True  # the flagDetect is set to true so that the bot is slowed down so that the depth value of the tomatoes can be accessed easily
            peri = cv2.arcLength(cnt, True)  # this is to get the perimeter of the contour

            approx = cv2.approxPolyDP(cnt, 0.02 * peri,
                                      True)  # 0.02 is a factor ---> it can be adjusted for better detection

            x, y, w, h = cv2.boundingRect(approx)  # this returns and stores the value of x, y, w, h

            cv2.circle(imgContour, (x + (w // 2), y + (h // 2)), max(h // 2, w // 2), (255, 0, 0),
                       2)  # this is the circle drawing function that draws image on imgContour with center x+(w/2) and y+(h/2) the next parameter is to get the radius of the circle that is the max of the entire spread
            cv2.circle(imgContour, (x + (w // 2), y + (h // 2)), 1, (255, 0, 0),
                       -1)  # the (255, 0, 0) ----> is in the BGR format so max of blue and 0 values of red and green makes the circle blue in colour
            midX = x + w // 2  # this is the mid of the circle
            midY = y + h // 2  # this is the mid of the circle

            if midX != 0 and midY != 0 and midX <= 640 and midY <= 480:  # as the frame of the output image is 640 x 480 pixels so this is to prevent accessing values that are out of bound by any case
                depth = depth_image[midY][midX]  # the depth value is registered for the point (midX, midY)

                depth = depth / 1000            #--------------> correction

                X = depth * ((midX - cx) / fx)  # conversion to the world coordinates
                Y = depth * ((midY - cy) / fy)  # conversion to the world coordinates
                Z = depth  # conversion to the world coordinates

                Z = Z*math.cos(0.10472)
                Y = Y - Z*math.sin(0.10472)

                br = tf2_ros.TransformBroadcaster()  # setting up the TF2 broadcaster
                t = geometry_msgs.msg.TransformStamped()  # broadcasting is stamped for every object
                t.header.stamp = rospy.Time.now()  # the head stamp is the current time that we use this makes it unique
                t.header.frame_id = "camera_link"  # as the camera on the arm has the camera_link2 so we are using that
                t.child_frame_id = "obj" + str(counter)  # this is the naming convention where the is given as obj + value of the counter -----> obj1, obj2 etc.

                cv2.putText(imgContour, t.child_frame_id,
                            # this function is used to put text on the imgContour, the text is the child_frame_id, at the point (midX, midY)
                            (midX, midY), cv2.FONT_HERSHEY_SIMPLEX,
                            # cv2.FONT_HERSHEY_SIMPLEX ----> is the font used to label the image
                            0.5, (255, 0,),
                            2)  # 0.5 is the font scale, (255, 0, 0) is for giving blue colour and 2 is the thickness

                ''' In this section we are trying to get the depth of the tomato visible ---> by doing this we will ensure that we accuate the arm only when the depth of the visible tomato
                is upto a certain limit so that we are unnecessarily don't try to pick up tomatoes on the other side of the plant and in doing so hit the plant and damage the arm'''

                # depth = depth_image[midY][midX]  # the depth value is re-adjusted so that we ensure that the depth value that we get is for the tomato and not the thing just around the tomato because we are not stopping to check the depth value but we are drifting with a small value
                print("Depth value ---------------------------------------------->: ", depth)
                if (depth <= 1.4) and (
                        flag_inp is True):  # over here the depth value that allows us to pick the tomatoes on the side where the bot is standing is 0.70

                    '''In this section we will only broadcast the TF when the tomato visible is in the depth range from where it can be picked'''

                    t.transform.translation.x = Z  # this is for transforming the world coordinates to the camera frame that is on the arm
                    t.transform.translation.y = -X  # this is for transforming the world coordinates to the camera frame that is on the arm
                    t.transform.translation.z = -Y  # this is for transforming the world coordinates to the camera frame that is on the arm

                    q = tf_conversions.transformations.quaternion_from_euler(0, 0,
                                                                             0)  # for conversion euler to quaternion
                    t.transform.rotation.x = q[0]
                    t.transform.rotation.y = q[1]
                    t.transform.rotation.z = q[2]
                    t.transform.rotation.w = q[3]

                    br.sendTransform(t)  # for broadcating the TF

                counter = counter + 1  # as this loop loops through the number of times equal to the number of unique contours that can be drawn then if the counter is incremented same number of times it will have unique value starting from 1 for every contour
                if counter > maxCounter:
                    maxCounter = counter
        else:
            flagDetect = False  # flagDetect is set to false if the tomato is not visible
            maxCounter = 0
    cv2.imshow("Result", imgContour)  # this is for displaying the final image with all the contours on it
    cv2.waitKey(1)  # this is for adding a 1 millisecond delay

    return x + w // 2, y + h // 2  # for mid of the box is returned using this




def laser_callback(msg):
    global regions
    global bleft
    global tripLaser
    bleft = min(msg.ranges[500:531])  # ------------> change this
    tripLaser = min(msg.ranges[500:515])  # ----> set this range also
    if bleft == 0:
        bleft = 50
    if tripLaser == 0:
        tripLaser = 50
    regions = {
        'bleft': min(bleft, 1.5),
        'tripLaser': min(tripLaser, 2.6),
        # ---------> adjust this value 2.5 ---> so that it is not triggered when it is turning with p controller but it gets triggered when it returns to the start position
    }


def Pcontroller():                          #-----> check the PController
    global angular_vel
    global linear_vel
    global regions
    kp = 1  # --> 1.5
    angular_vel = kp * (regions['bleft'] - 0.63)
    # edit
    # if angular_vel > 0.8:
    #     angular_vel = 0.2
    # edit
    # if abs(regions['bleft'] - 0.60) > 0.2:
    #     linear_vel = 0.1

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


def main(args):  # this is the main method

    # rospy.init_node('object_detection_manipulation', anonymous=True)  # for initializing the node with name object_detection
    # tfBuffer = tf2_ros.Buffer()  # for setting the buffer
    # listener = tf2_ros.TransformListener(tfBuffer)  # for using the TF listener
    # ur5 = Ur5Moveit()  # for setting up the ur5 object of Ur5Moveit class
    # depth_sub = rospy.Subscriber("/camera/depth/image_raw2", Image, callback_depth)  # for setting the depth image subscriber
    # image_sub = rospy.Subscriber("/camera/color/image_raw2", Image, callback)  # for setting the color image subscriber
    # rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)  # setting up the subscriber
    # # rospy.Subscriber('/odom', Odometry, odom_callback)  # setting up the subscriber
    # pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # setting up the publisher



    rospy.init_node('object_detection_manipulation',
                    anonymous=True)  # for initializing the node with name object_detection
    tfBuffer = tf2_ros.Buffer()  # for setting the buffer
    listener = tf2_ros.TransformListener(tfBuffer)  # for using the TF listener
    # ur5 = Ur5Moveit()  # for setting up the ur5 object of Ur5Moveit class
    # image_sub_aruco = rospy.Subscriber("/ebot/camera1/image_raw", Image, callback_Aruco)

    sub_color_image = rospy.Subscriber(
        "/camera/aligned_depth_to_color/image_raw", Image, image_dp, queue_size=100)

    sub_depth_image = rospy.Subscriber("/camera/color/image_raw/compressed",
                                       CompressedImage, image_cb, queue_size=100)

    rospy.Subscriber('/scan', LaserScan, laser_callback)  # setting up the subscriber
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)  # setting up the publisher



    ur5_pose_1 = geometry_msgs.msg.Pose()  # for using the pose
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
    global flag_inp
    global flagCheck
    flagNewPose = False  # to get to a new pose to make the accuation of the arm from this new pose in case it fails the first time

    hold = ''

    poseTomatoY = ''

    flagStage0 = False
    flagIntermediateFirst = False
    flagIntermediateSecond = False
    flagIntermediateThird = False
    flagIntermediateLast = False
    start_time = time.time()
    seconds = 3
    delay = 23
    delay2 = 5
    rate = rospy.Rate(3)
    while not rospy.is_shutdown():  # while loop
        current_time = time.time()
        elapsed_time = current_time - start_time
        # print("Laser Value : ",regions['bleft'])
        if elapsed_time >= seconds:
            if flagEndMotion is True:  # gets triggered when the bot returns to its start location after completing the motion
                stopMotion()  # to stop the motion when the allignment of the aruco
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
                rospy.loginfo('Mission Accomplished!')
                break

            #####FOR TAKING THE ARM TO THE PLANT PERCEPTION POSE########
            if flagPose is True:
                flagPose = False
                # ur5.arm_robot(2)
                # set_joint(1)
                # set_pose()
                rospy.sleep(0.1)
                flag = True
                flag_inp = True

            ##########################################################
            elif flag is True:

                for i in range(
                        maxCounter + 1):  # this loop actually checks which tomato is in the permissible depth range
                    try:
                        trans = tfBuffer.lookup_transform('base_link', 'obj' + str(i), rospy.Time(0))  # for getting the TF of obj0 ----> tomato in the scene
                        hold = str(i)
                        # print("%%%%%%%%%%%%%%%%%%%%%%%%%",hold)
                    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                        continue

                    if hold != '':
                        break

                if hold == '':  # if no tomato is visible the hold will be blank string so the bot will keep moving
                    '''
                    if (flagStage0 is False):
                        flagStage0 = True
                        while regions['bleft'] > 1.0:
                            rospy.loginfo(1)
                            forward()
                            velocity_msg.linear.x = linear_vel
                            velocity_msg.angular.z = angular_vel
                            pub.publish(velocity_msg)
                            rospy.sleep(0.1)
        
                        forward()
                        rospy.loginfo(2)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(6)
    
                    elif (regions['tripLaser'] >= 2.5) and (flagIntermediateFirst is False):
                        flagIntermediateFirst = True
                        forward()
                        rospy.loginfo(3)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(delay2)
        
                        initial_orient = orient
        
                        while rotation < 1.57:
                            # print(rotation)
                            if abs(orient - initial_orient) < 1:
                                rotation = rotation + abs(orient - initial_orient)
                            initial_orient = orient
                            velocity_msg.linear.x = 0.0
                            velocity_msg.angular.z = 0.2
                            pub.publish(velocity_msg)
        
                        rotation = 0.0
        
                        forward()
                        rospy.loginfo(4)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(8.5)
        
                        initial_orient = orient
        
                        while rotation < 1.57:
                            # print(rotation)
                            if abs(orient - initial_orient) < 1:
                                rotation = rotation + abs(orient - initial_orient)
                            initial_orient = orient
                            velocity_msg.linear.x = 0.0
                            velocity_msg.angular.z = 0.2
                            pub.publish(velocity_msg)
        
                        rotation = 0.0
        
                        forward()
                        rospy.loginfo(5)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(4)
        
                    elif (regions['tripLaser'] >= 2.5) and (flagIntermediateSecond is False):
                        flagIntermediateSecond = True
                        forward()
                        rospy.loginfo(6)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(delay2)
        
                        initial_orient = orient
        
                        while rotation < 1.57:
                            # print(rotation)
                            if abs(orient - initial_orient) < 1:
                                rotation = rotation + abs(orient - initial_orient)
                            initial_orient = orient
                            velocity_msg.linear.x = 0.0
                            velocity_msg.angular.z = 0.2
                            pub.publish(velocity_msg)
        
                        rotation = 0.0
        
                        forward()
                        rospy.loginfo(7)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(18)
        
                        initial_orient = orient
        
                        while rotation < 1.57:
                            # print(rotation)
                            if abs(orient - initial_orient) < 1:
                                rotation = rotation + abs(orient - initial_orient)
                            initial_orient = orient
                            velocity_msg.linear.x = 0.0
                            velocity_msg.angular.z = 0.2
                            pub.publish(velocity_msg)
        
                        rotation = 0.0
        
                        forward()
                        rospy.loginfo(8)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(4)
        
        
                    elif (regions['tripLaser'] >= 2.5) and (flagIntermediateThird is False):
                        flagIntermediateThird = True
                        forward()
                        rospy.loginfo(9)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(delay2)
        
                        initial_orient = orient
        
                        while rotation < 1.57:
                            # print(rotation)
                            if abs(orient - initial_orient) < 1:
                                rotation = rotation + abs(orient - initial_orient)
                            initial_orient = orient
                            velocity_msg.linear.x = 0.0
                            velocity_msg.angular.z = 0.2
                            pub.publish(velocity_msg)
        
                        rotation = 0.0
        
                        forward()
                        rospy.loginfo(10)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(7)
        
                        initial_orient = orient
        
                        while rotation < 1.57:
                            # print(rotation)
                            if abs(orient - initial_orient) < 1:
                                rotation = rotation + abs(orient - initial_orient)
                            initial_orient = orient
                            velocity_msg.linear.x = 0.0
                            velocity_msg.angular.z = 0.2
                            pub.publish(velocity_msg)
        
                        rotation = 0.0
        
                        forward()
                        rospy.loginfo(11)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(4)
        
        
                    elif (regions['tripLaser'] >= 2.5) and (flagIntermediateLast is False):
                        flagIntermediateLast = True
                        forward()
                        rospy.loginfo(12)
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(delay2)
        
                        stopMotion()
                        velocity_msg.linear.x = linear_vel
                        velocity_msg.angular.z = angular_vel
                        pub.publish(velocity_msg)
                        rospy.sleep(0.1)
                        flagEndMotion = True
                        continue

                    forward()
                    Pcontroller()
                    rospy.loginfo(13)
                    if flagDetect is True:
                        velocity_msg.linear.x = linear_vel - 0.1
                    else:
                        velocity_msg.linear.x = linear_vel
                    velocity_msg.angular.z = angular_vel
                    pub.publish(velocity_msg)
                    rospy.sleep(0.1)

                    '''

                    ArucoId = '-1'  # set to invalid id -1

                    continue  # to continue with the next iteration if we get some exception

                ##########################################################################################################

                try:
                    transAruco = tfBuffer.lookup_transform('camera_link', 'obj' + hold, rospy.Time(0))
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    velocity_msg.linear.x = 0.1  # till the time the TF of the aruco is not available then the bot is moving with a small velocity of 0.1
                    velocity_msg.angular.z = 0.0
                    pub.publish(velocity_msg)
                    continue

                # print("x  : ",transAruco.transform.translation.x)
                # print("y  : ", transAruco.transform.translation.y)
                # print("z  : ", transAruco.transform.translation.z)
                #
                # print("$$$$$$$$$$$$$$$$$$$$$$$$$$",hold)
                # while transAruco.transform.translation.y > 0.01 or transAruco.transform.translation.y < -0.01:  # when the TF is available but the center of the Aruco is not in the range
                #     if transAruco.transform.translation.y > 0.01:  # if the bot has moved forward by any chance then a negative velocity will bring it back
                #         velocity_msg.linear.x = -0.1
                #         velocity_msg.angular.z = 0.0
                #         pub.publish(velocity_msg)
                #     elif transAruco.transform.translation.y < -0.01:  # if the bot has moved backward then a positive velocity will do the work
                #         velocity_msg.linear.x = 0.1
                #         velocity_msg.angular.z = 0.0
                #         pub.publish(velocity_msg)
                #     transAruco = tfBuffer.lookup_transform('camera_link', 'obj' + hold, rospy.Time(0))  # the TF is continuously read so that we are able to allign the aruco properly
                print("x  : ", transAruco.transform.translation.x)
                print("y  : ", transAruco.transform.translation.y)
                print("z  : ", transAruco.transform.translation.z)

                ##########################################################################################################

                stopMotion()  # to stop the motion when the allignment of the aruco
                velocity_msg.linear.x = linear_vel
                velocity_msg.angular.z = angular_vel
                pub.publish(velocity_msg)
                rospy.sleep(0.1)

                trans = tfBuffer.lookup_transform('base_link', 'obj' + hold, rospy.Time(0))  # the TF of the tomato is read again when the bot is completely stopped so that the TF of the tomato is perfectly correct

                # rospy.loginfo('obj' + hold + ' Identified at ' + ArucoId)

                # ************************************************************VARIABLE DEFINITIONS******************************************************#

                flag_inp = False
                rospy.sleep(0.1)
                tomato_Position_X_Transform = trans.transform.translation.x  # for storing the x value of transform for the tomato 2
                tomato_Position_Y_Transform = trans.transform.translation.y  # for storing the y value of transform for the tomato 2
                tomato_Position_Z_Transform = trans.transform.translation.z  # for storing the z value of transform for the tomato 2

                base_link_Position_X = 0.16 # this is the x position of the base_link ----> our reference point
                base_link_Position_Y = 0.0  # this is the y position of the base_link ----> our reference point
                base_link_Position_Z = 0.53 # this is the z position of the base_link ----> our reference point

                offset_x_1 = 0.16
                offset_y_1 = 0.4
                offset_z_1 = 0.02

                offset_x_2 = 0.1
                offset_y_2 = 0.25
                #
                ur5_pose_1.orientation.x = -0.20426466049594807
                ur5_pose_1.orientation.y = 0.9759094471343439
                ur5_pose_1.orientation.z = -0.07502044797426752
                ur5_pose_1.orientation.w = 0.01576806431223178
                #
                ur5_pose_1.position.x = base_link_Position_X + tomato_Position_X_Transform + offset_x_1
                ur5_pose_1.position.y = base_link_Position_Y + tomato_Position_Y_Transform - offset_y_1
                ur5_pose_1.position.z = base_link_Position_Z + tomato_Position_Z_Transform

                if(ur5_pose_1.position.x == 0 or ur5_pose_1.position.y == 0 or ur5_pose_1.position.z == 0):
                    continue
                else:
                    set_pose(ur5_pose_1.position.x, ur5_pose_1.position.y, ur5_pose_1.position.z,  0.5473376689156836, 0.43518671591792285, 0.5518910850218367, 0.4543679442345587)


                rospy.sleep(0.1)
                #
                # ur5_pose_1.position.x = base_link_Position_X + tomato_Position_X_Transform + offset_x_2
                # ur5_pose_1.position.y = base_link_Position_Y + tomato_Position_Y_Transform - offset_y_2
                # ur5_pose_1.position.z = base_link_Position_Z + tomato_Position_Z_Transform + offset_z_1
                #
                # set_pose(ur5_pose_1.position.x, ur5_pose_1.position.y, ur5_pose_1.position.z, 0, 0, 0, 1)
                #
                # rospy.sleep(0.1)
                # 
                #
                # close_gripper()
                # rospy.sleep(0.1)
                #
                # set_joint(2)
                # rospy.sleep(0.1)
                #
                # open_gripper()
                # rospy.sleep(0.1)

                # ur5_pose_1.position.x = base_link_Position_X + tomato_Position_X_Transform
                # ur5_pose_1.position.y = base_link_Position_Y + tomato_Position_Y_Transform
                # ur5_pose_1.position.z = base_link_Position_Z + tomato_Position_Z_Transform
                #
                # # poseTomatoY = ur5_pose_1.position.y
                #
                # cartesian_plan, fraction = ur5.plan_cartesian_path(1, ur5_pose_1.position.x, ur5_pose_1.position.y,
                #                                                    ur5_pose_1.position.z)
                # ur5.display_trajectory(cartesian_plan)
                # ur5.execute_plan(cartesian_plan)
                #
                # # set_pose(ur5_pose_1.position.x, ur5_pose_1.position.y, ur5_pose_1.position.z, 0, 0 ,0, 1)
                # # close_gripper() # state 1 of the gripper is to close the gripper
                # # rospy.sleep(0.1)
                #
                # ur5.arm_robot(1)  # to bring the arm to the bucket
                # # set_joint(2)
                # rospy.sleep(0.1)
                # open_gripper()  # to open the gripper
                # rospy.sleep(0.1)

                ######################################################################################################

                flagPose = True  # to set this variable to true again so that the arm positions itself correctly after picking up the tomato
                flag = False  # this will prevent further execution till the arm has not positioned itself correctly
                hold = ''
                flagNewPose = False
                flagCheck = False

            rate.sleep()

    # del ur5  # to delete the ur5 object


if __name__ == '__main__':
    main(sys.argv)  # for calling the main method