#!/usr/bin/env python

import argparse
import rospy

from controller_manager_msgs.srv import *
from rosservice import rosservice_find
from urdf_parser_py.urdf import URDF

parser = argparse.ArgumentParser(usage='Print joint limits')
parser.add_argument('-c', '--controller', default=None, help='Controller name')
parser.add_argument('-j', '--joint', default=None, help='Joint name')
args = parser.parse_args()

robot = URDF.from_parameter_server()

if args.controller is not None:
    controller_managers = rosservice_find('controller_manager_msgs/ListControllers')
    for cm in controller_managers:
        rospy.wait_for_service(cm)
        try:
            list_controllers = rospy.ServiceProxy(cm, ListControllers)
            controller_list = list_controllers()
            for c in controller_list.controller:
                if c.name == args.controller:
                    print('\n')
                    print (args.controller)
                    for j in c.resources:
                        limit = robot.joint_map[j].limit
                        print (j + '\t' + str([limit.lower, limit.upper]))
                        
        except rospy.ServiceException as e:
            print ("Service call failed: %s"%e)

if args.joint is not None:
    limit = robot.joint_map[args.joint].limit
    print('\n')
    print (args.joint)
    print ([limit.lower, limit.upper])