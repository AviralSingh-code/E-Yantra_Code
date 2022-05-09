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
from math import pi
from std_msgs.msg import String
from moveit_commander.conversions import pose_to_list
import copy



######## COLOR THRESHOLDING AND FILTERING ########
'''
h_min ----> Min value of Hue
s_min ----> Min value of Saturation
v_min ----> Min value of Value
h_max ----> Max value of Hue
s_max ----> Max value of Saturation
v_max ----> Max value of Value
'''

tomatoColor = [[0, 37, 117, 0, 250, 255]]       #Defining the HSV color space of red tomatoes with format [h_min, s_min, v_min, h_max, s_max, v_max]

###################################################


### GLOBAL VARIABLE ####

imgContour = None                           #imgContour is declared of the type None and it will hold the final image
depth_image = np.empty((480,640),float)     #this is a 2D array that hold the depth value of every pixel of the frame
midX = 0                                #this is the middle value of the entire span of the contour in the x - axis
midY = 0                        #this is the middle value of the entire span of the contour in the y - axis
linear_vel = 0.0
angular_vel = 0.0

maxCounter = 0

regions = {                 #this is to hold the laser values of paricular regions
        'bleft':  0.0  ,
    }


flagPose = True         # this flag is used to first set the arm to a particular pose before starting the detection of the tomatoes 
flag = False           #this is to start detection and motion of the bot once the arm is set to a particular pose

flagEndMotion = False

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
        rospy.init_node('move_group_python_interface_tutorial',anonymous=True)
        robot = moveit_commander.RobotCommander()
        scene = moveit_commander.PlanningSceneInterface()
        group_name = "arm"
        group = moveit_commander.MoveGroupCommander(group_name)


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


    def plan_cartesian_path(self, scale, value):
        group = self.group


        waypoints = []

        # wpose = group.get_current_pose().pose
        # wpose.position.z += scale * 0.3  # First move up (z)
        # wpose.position.y += scale * 0.2  # and sideways (y)
        # waypoints.append(copy.deepcopy(wpose))

        wpose = group.get_current_pose().pose
        wpose.position.z += value  # First move up (z)
        # wpose.position.y += scale * 0.2  # and sideways (y)
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



    # def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    #     # Copy class variables to local variables to make the web tutorials more clear.
    #     # In practice, you should use the class variables directly unless you have a good
    #     # reason not to.
    #     box_name = self.box_name
    #     scene = self.scene

    
    #     start = rospy.get_time()
    #     seconds = rospy.get_time()
    #     while (seconds - start < timeout) and not rospy.is_shutdown():
    #         # Test if the box is in attached objects
    #         attached_objects = scene.get_attached_objects([box_name])
    #         is_attached = len(attached_objects.keys()) > 0

    #         # Test if the box is in the scene.
    #         # Note that attaching the box will remove it from known_objects
    #         is_known = box_name in scene.get_known_object_names()

    #         # Test if we are in the expected state
    #         if (box_is_attached == is_attached) and (box_is_known == is_known):
    #           return True

    #         # Sleep so that we give other threads time on the processor
    #         rospy.sleep(0.1)
    #         seconds = rospy.get_time()

    #     return False




def main():
    # waypoints = []
    # wpose = group.get_current_pose().pose
    # wpose.position.z += scale * 0.3  # First move up (z)
    # wpose.position.y += scale * 0.2  # and sideways (y)
    # waypoints.append(copy.deepcopy(wpose))
    # waypoints.append(1*0.3)
    try:
        print ("============ Press `Enter` to plan and display a Cartesian path ...")
        # raw_input()
        tutorial = Ur5Moveit()
        cartesian_plan, fraction = tutorial.plan_cartesian_path(1, 1*0.3)
        print ("============ Press `Enter` to execute a saved path ...")
        # raw_input()
        tutorial.display_trajectory(cartesian_plan)
        # tutorial.execute_plan(cartesian_plan)
    except rospy.ROSInterruptException:
        return
    except KeyboardInterrupt:
        return

if __name__ == '__main__':
  main()

