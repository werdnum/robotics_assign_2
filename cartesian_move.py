#! /usr/bin/env python

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys
import numpy as np

import actionlib
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

if __name__ == '__main__':
    try:
        rospy.init_node('jaco_cartesian_move')

	client = actionlib.SimpleActionClient('/jaco_arm_driver/arm_pose/arm_pose', jaco_msgs.msg.ArmPoseAction)
    	client.wait_for_server()

    	goal = jaco_msgs.msg.ArmPoseGoal()
    	goal.pose.header = std_msgs.msg.Header(frame_id=('jaco_api_origin'))
    	goal.pose.pose.position = geometry_msgs.msg.Point(
        	float(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]))
    	goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
        	float(sys.argv[4]), float(sys.argv[5]), float(sys.argv[6]), float(sys.argv[7]))

    	client.send_goal(goal)

    	if not client.wait_for_result(rospy.Duration(5.0)):
        	client.cancel_all_goals()
        	print('        the cartesian action timed-out')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
