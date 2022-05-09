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

regions = {
    'bleft': 0.0,
    'tripLaser': 0.0,
}

bleft = 0.0
tripLaser = 0.0
orient = 10.0
init_orient = 10.0
init_flag = False

def laser_callback(msg):
    global regions
    global bleft
    global tripLaser
    bleft = min(msg.ranges[500:531])                    #------------> change this
    tripLaser = min(msg.ranges[500:515])      #----> set this range also
    if bleft == 0:
        bleft = 50
    if tripLaser == 0:
        tripLaser = 50
    regions = {
        'bleft': min(bleft, 1.5),
        'tripLaser': min(tripLaser, 2.6),               #---------> adjust this value 2.5 ---> so that it is not triggered when it is turning with p controller but it gets triggered when it returns to the start position
    }

def Pcontroller():                          #-----> check the PController
    global angular_vel
    global linear_vel
    global regions
    kp = 2  # --> 1.5
    angular_vel = kp * (regions['bleft'] - 0.63)
    # edit
    if angular_vel > 0.8:
        angular_vel = 0.3
    # edit
    # if abs(regions['bleft'] - 0.60) > 0.2:
    #     linear_vel = 0.1


def imu_callback(msg):
    global orient
    global initial_orient
    # global flag_intial_Orient
    orientation_q = msg.orientation

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]

    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    if yaw < 0:
        yaw = 3.1369 + (3.1369 + yaw)

    orient = yaw
    # if flag_intial_Orient is False:
    #     initial_orient = yaw
    #     flag_intial_Orient = True

    print("Orientation using Imu is : ", orient)

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


def main():
    rospy.init_node('object_detection_manipulation', anonymous=True)
    rospy.Subscriber('/scan', LaserScan, laser_callback)  # setting up the subscriber     -------------------> change the topic over here as well for the laser function
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/imu', Imu, imu_callback)
    
    velocity_msg = Twist()
    velocity_msg.linear.x = 0
    velocity_msg.linear.y = 0
    pub.publish(velocity_msg)

    global linear_vel
    global angular_vel
    global regions
    global orient
    global init_orient
    delay = 16
    delay2 = 5
    rotation = 0.0

    rate = rospy.Rate(5)
    flagStage0 = False
    flagIntermediateFirst = False
    flagIntermediateSecond = False
    flagIntermediateThird = False
    flagIntermediateLast = False
    start_time = time.time()
    seconds = 3
    while not rospy.is_shutdown():
        current_time = time.time()
        elapsed_time = current_time - start_time
        print(regions['bleft'])
        if elapsed_time >= seconds:
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
                break

            forward()
            Pcontroller()
            rospy.loginfo(13)
            velocity_msg.linear.x = linear_vel
            velocity_msg.angular.z = angular_vel
            pub.publish(velocity_msg)
            rospy.sleep(0.1)

        rate.sleep()


if __name__ == '__main__':
    main()


