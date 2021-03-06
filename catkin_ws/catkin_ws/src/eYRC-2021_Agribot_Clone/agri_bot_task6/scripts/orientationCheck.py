#!/usr/bin/env python3


import sys  # module to handle runtime environment
import rospy  # for use of pytho with ROS
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry
import geometry_msgs.msg  # for geometry messages
import actionlib  # for importing for providing standard interface e.g. for returning point cloud
import math  # for importing the math library


pose = [0.0] * 4

def odom_callback(data):  # callback funtion for odom to get the correct position of the bot
    global pose
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x, y, z, w])[2]]



def main():
    rospy.init_node('object_detection_manipulation', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)  # setting up the subscriber

    rate = rospy.Rate(10)

    global pose

    while not rospy.is_shutdown():

        rospy.loginfo(pose[2])

        rate.sleep()

if __name__=='__main__':
    main()
