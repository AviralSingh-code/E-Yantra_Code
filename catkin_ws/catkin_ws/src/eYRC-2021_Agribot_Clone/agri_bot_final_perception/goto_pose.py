#! /usr/bin/env python

import rospy
import sys
import copy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib
import math
import tf





def main():

    ur5 = Ur5Moveit()


    
    # ur5_pose_1.position.x = 0.05084966766295753
    # ur5_pose_1.position.y = -0.3010083745944882 
    # ur5_pose_1.position.z = 1.0103444390397311 
    # ur5_pose_1.orientation.x = 4.144230960006712e-05
    # ur5_pose_1.orientation.y = 0.9990605558490008
    # ur5_pose_1.orientation.z = -0.04333591551294594
    # ur5_pose_1.orientation.w = 4.955679019632736e-05



    # ur5_pose_1.position.x = -0.04619852924724534
    # ur5_pose_1.position.y = 0.16996051819939506 
    # ur5_pose_1.position.z = 1.1967762937196476 
    # ur5_pose_1.orientation.x = 0.22824103136299792
    # ur5_pose_1.orientation.y = -0.9731578255451082
    # ur5_pose_1.orientation.z = -0.02872059735018363
    # ur5_pose_1.orientation.w = 0.00670861166862685

    # ur5_pose_1.position.x = 0.053850973865922344 + 0.726
    # ur5_pose_1.position.y = -0.3469873568466747 + 0.211
    # ur5_pose_1.position.z = 1.0148560493915084 + 0.013
    # ur5_pose_1.orientation.x = 0.004334978797122511 - 1.000
    # ur5_pose_1.orientation.y = -0.9996467569585759 + 0.000
    # ur5_pose_1.orientation.z = -0.02622137940371337 + 0.000
    # ur5_pose_1.orientation.w = 9.23211622507888e-05 + 0.001

    # ur5_pose_1.position.x = 0.053850973865922344 + 0.726
    # ur5_pose_1.position.y = -0.3469873568466747 + 0.211
    # ur5_pose_1.position.z = 1.0148560493915084 + 0.013
    # ur5_pose_1.orientation.x = 0.004334978797122511 - 1.000
    # ur5_pose_1.orientation.y = -0.9996467569585759 + 0.000
    # ur5_pose_1.orientation.z = -0.02622137940371337 + 0.000
    # ur5_pose_1.orientation.w = 9.23211622507888e-05 + 0.001

    
    
    while not rospy.is_shutdown():
        

        

    


if __name__ == '__main__':
    main()
