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



def odom_callback(data):
    global orient
    x = data.pose.pose.orientation.x
    y = data.pose.pose.orientation.y
    z = data.pose.pose.orientation.z
    w = data.pose.pose.orientation.w
    pose = [data.pose.pose.position.x, data.pose.pose.position.y, euler_from_quaternion([x, y, z, w])[2]]
    # rospy.loginfo(pose[2])
    orient = pose[2]


def imu_callback(msg):
    orientation_q = msg.orientation

    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

    if yaw < 0:
        yaw = 3.1369 + (3.1369 + yaw)
        
    print("Orientation using Imu is : ",yaw)

def main():
    rospy.init_node('object_detection_manipulation', anonymous=True)
    rospy.Subscriber('/odom', Odometry, odom_callback)  # setting up the subscriber
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
    rate = rospy.Rate(5)
    start_time = time.time()
    seconds = 4
    flagStage0 = False
    flagIntermediate = False
    flagLastStage = False
    flagIntermediateFirst = False
    flagIntermediateSecond = False
    while not rospy.is_shutdown():
        # current_time = time.time()
        # elapsed_time = current_time - start_time
        # if elapsed_time >= seconds:
        #     if (0.5 < orient < 2.5) and (flagStage0 is False):
        #         flagStage0 = True
        #         while regions['bleft'] > 1.0:
        #             rospy.loginfo(1)
        #             forward()
        #             velocity_msg.linear.x = linear_vel
        #             velocity_msg.angular.z = angular_vel
        #             pub.publish(velocity_msg)
        #             rospy.sleep(0.1)

        #         forward()
        #         rospy.loginfo(6)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(2)

        #     elif (0.5 < orient < 2.5) and (regions['tripLaser'] >= 2.4) and (flagIntermediateFirst is False):
        #         flagIntermediateFirst = True
        #         forward()
        #         rospy.loginfo(10)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(4)

        #         if orient > 0:
        #             while orient <= 3:
        #                 rospy.loginfo(12)
        #                 linear_vel = 0.0
        #                 angular_vel = 0.2
        #                 velocity_msg.linear.x = linear_vel
        #                 velocity_msg.angular.z = angular_vel
        #                 pub.publish(velocity_msg)
        #                 rospy.sleep(0.1)
        #         elif orient < 0:
        #             while orient >= -3:
        #                 rospy.loginfo(13)
        #                 linear_vel = 0.0
        #                 angular_vel = -0.2
        #                 velocity_msg.linear.x = linear_vel
        #                 velocity_msg.angular.z = angular_vel
        #                 pub.publish(velocity_msg)
        #                 rospy.sleep(0.1)

        #         forward()
        #         rospy.loginfo(14)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(9)

        #         while orient <= -1.6 or orient >= -1.5:
        #             rospy.loginfo(15)
        #             linear_vel = 0.0
        #             angular_vel = 0.2
        #             velocity_msg.linear.x = linear_vel
        #             velocity_msg.angular.z = angular_vel
        #             pub.publish(velocity_msg)
        #             rospy.sleep(0.1)


        #         forward()
        #         rospy.loginfo(16)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(4)


        #     elif (-2.5 < orient < -0.5) and (regions['tripLaser'] >= 2.4) and (flagIntermediate is False):       #-----> check the tripLaser value
        #         flagIntermediate = True
        #         forward()
        #         rospy.loginfo(10)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(4)

        #         if orient > 0.15:
        #             while orient > 0.15:
        #                 rospy.loginfo(8)
        #                 linear_vel = 0.0
        #                 angular_vel = -0.2
        #                 velocity_msg.linear.x = linear_vel
        #                 velocity_msg.angular.z = angular_vel
        #                 pub.publish(velocity_msg)
        #                 rospy.sleep(0.1)
        #         elif orient < -0.15:
        #             while orient < -0.15:
        #                 rospy.loginfo(9)
        #                 linear_vel = 0.0
        #                 angular_vel = 0.2
        #                 velocity_msg.linear.x = linear_vel
        #                 velocity_msg.angular.z = angular_vel
        #                 pub.publish(velocity_msg)
        #                 rospy.sleep(0.1)


        #         forward()
        #         rospy.loginfo(3)
        #         velocity_msg.linear.x = linear_vel                #-----> check the linear velocity
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(17)                              #--------> check this

        #         while orient < 1.57:
        #             rospy.loginfo(44)
        #             linear_vel = 0.0
        #             angular_vel = 0.2
        #             velocity_msg.linear.x = linear_vel
        #             velocity_msg.angular.z = angular_vel
        #             pub.publish(velocity_msg)
        #             rospy.sleep(0.1)


        #         forward()
        #         rospy.loginfo(3)
        #         velocity_msg.linear.x = linear_vel                #-----> check the linear velocity
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(6) 


        #     elif (0.5 < orient < 2.5) and (regions['tripLaser'] >= 2.4) and (flagIntermediateSecond is False) and (flagIntermediateFirst is True):
        #         flagIntermediateSecond = True
        #         forward()
        #         rospy.loginfo(10)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(4)

        #         if orient > 0:
        #             while orient <= 3:
        #                 rospy.loginfo(12)
        #                 linear_vel = 0.0
        #                 angular_vel = 0.2
        #                 velocity_msg.linear.x = linear_vel
        #                 velocity_msg.angular.z = angular_vel
        #                 pub.publish(velocity_msg)
        #                 rospy.sleep(0.1)
        #         elif orient < 0:
        #             while orient >= -3:
        #                 rospy.loginfo(13)
        #                 linear_vel = 0.0
        #                 angular_vel = -0.2
        #                 velocity_msg.linear.x = linear_vel
        #                 velocity_msg.angular.z = angular_vel
        #                 pub.publish(velocity_msg)
        #                 rospy.sleep(0.1)

        #         forward()
        #         rospy.loginfo(14)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(9)

        #         while orient <= -1.6 or orient >= -1.5:
        #             rospy.loginfo(15)
        #             linear_vel = 0.0
        #             angular_vel = 0.2
        #             velocity_msg.linear.x = linear_vel
        #             velocity_msg.angular.z = angular_vel
        #             pub.publish(velocity_msg)
        #             rospy.sleep(0.1)


        #         forward()
        #         rospy.loginfo(16)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(4)


            
            

        #     elif (-2.5 < orient < -0.5) and (regions['tripLaser'] >= 2.4) and (flagLastStage is False):
        #         flagLastStage = True
        #         forward()
        #         rospy.loginfo(13)
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(4)                              #-----> check this value 

        #         stopMotion()
        #         velocity_msg.linear.x = linear_vel
        #         velocity_msg.angular.z = angular_vel
        #         pub.publish(velocity_msg)
        #         rospy.sleep(0.1)

        #         break



        #     forward()
        #     Pcontroller()
        #     rospy.loginfo(5)
        #     velocity_msg.linear.x = linear_vel
        #     velocity_msg.angular.z = angular_vel
        #     pub.publish(velocity_msg)
        #     rospy.sleep(0.1)

        rate.sleep()


if __name__ == '__main__':
    main()