#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys
import numpy as np

import actionlib
import jaco_msgs.msg
import std_msgs.msg
import geometry_msgs.msg

from assign2.srv import Cartesian

def handle_cartesian_move (req):
    try:
        #rospy.init_node('jaco_cartesian_move')

        client = actionlib.SimpleActionClient('/jaco_arm_driver/arm_pose/arm_pose', jaco_msgs.msg.ArmPoseAction)
        client.wait_for_server()

        goal = jaco_msgs.msg.ArmPoseGoal()
        goal.pose.header = std_msgs.msg.Header(frame_id=('jaco_api_origin'))
        goal.pose.pose.position = geometry_msgs.msg.Point(
            req.pos1, req.pos2, req.pos3)
        goal.pose.pose.orientation = geometry_msgs.msg.Quaternion(
            req.orient1, req.orient2, req.orient3, req.orient4)

        client.send_goal(goal)

        if not client.wait_for_result(rospy.Duration(5.0)):
            client.cancel_all_goals()
            print('        the cartesian action timed-out')

        return True

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

def cartesian_move_server():
    rospy.init_node('jaco_cartesian_move')
    s = rospy.Service('cartesian_move', Cartesian, handle_cartesian_move)
    rospy.spin()

if __name__ == '__main__':
    cartesian_move_server()

#if __name__ == '__main__':
#   main(sys.argv[1], sys.argv[2], sys.argv[3], sys.argv[4], sys.argv[5], sys.argv[6], sys.argv[7])
