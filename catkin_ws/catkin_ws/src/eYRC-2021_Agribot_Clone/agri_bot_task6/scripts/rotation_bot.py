#!/usr/bin/env python3


import sys  # module to handle runtime environment
import rospy  # for use of pytho with ROS
from geometry_msgs.msg import Twist

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import geometry_msgs.msg  # for geometry messages
import actionlib  # for importing for providing standard interface e.g. for returning point cloud
import math  # for importing the math library
import time
from sensor_msgs.msg import Imu     #-------> for Imu


linear_vel = 0.0
angular_vel = 0.0

flag_intial_Orient = False
initial_orient = 10.0

regions = {
    'bleft': 0.0,
    'bright': 0.0,
}

bleft = 0.0
tripLaser = 0.0
orient = 10.0
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



# def odom_callback(data):
#     global orient
#     x = data.pose.pose.orientation.x
#     y = data.pose.pose.orientation.y
#     z = data.pose.pose.orientation.z
#     w = data.pose.pose.orientation.w
#     pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x, y, z, w])[2]]
#     # rospy.loginfo(pose[2])
#     orient = pose[2]


def imu_callback(msg):
    global orient
    global initial_orient
    global flag_intial_Orient
    orientation_q = msg.orientation

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    if yaw < 0:
        yaw = 3.1369 + (3.1369 + yaw)

    orient = yaw
    if flag_intial_Orient is False:
        initial_orient = yaw
        flag_intial_Orient = True
    
        
    # print("Orientation using Imu is : ",initial_orient)

def main():
    rospy.init_node('object_detection_manipulation', anonymous=True)
    # rospy.Subscriber('/odom', Odometry, odom_callback)  # setting up the subscriber
    rospy.Subscriber('/imu', Imu, imu_callback)  # setting up the subscriber
    rospy.Subscriber('/ebot/laser/scan', LaserScan, laser_callback)  # setting up the subscriber     -------------------> change the topic over here as well for the laser function
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    pub.publish(velocity_msg)
    global linear_vel
    global angular_vel
    global regions
    global orient
    global initial_orient
    rate = rospy.Rate(5)
    start_time = time.time()
    seconds = 4
    flagStage0 = False
    flagIntermediate = False
    flagLastStage = False
    flagIntermediateFirst = False
    flagIntermediateSecond = False
    rotation = 0.0
    flag_orient_trip = False
    max_rot = 0.0
    min_rot = 0.0
    max_Yaw = initial_orient
    
    while not rospy.is_shutdown():
        while rotation < 1.57:
            print(rotation)
            if abs(orient - initial_orient) < 1: 
                rotation = rotation + abs(orient - initial_orient)
            initial_orient = orient
            velocity_msg.linear.x = 0.0
            velocity_msg.angular.z = 0.2
            pub.publish(velocity_msg)

        print("Over")
        velocity_msg.linear.x = 0.0
        velocity_msg.angular.z = 0.0
        pub.publish(velocity_msg)
        rate.sleep()


if __name__ == '__main__':
    main()