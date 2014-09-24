#! /usr/bin/env python

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys

import actionlib
import jaco_msgs.msg

if __name__ == '__main__':

    try:
       	rospy.init_node('jaco_finger_move')

       	action_address = '/jaco_arm_driver/fingers/finger_positions'
    	client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.SetFingersPositionAction)

    	client.wait_for_server()

    	goal = jaco_msgs.msg.SetFingersPositionGoal()
    	goal.fingers.finger1 = float(sys.argv[1])
    	goal.fingers.finger2 = float(sys.argv[2])
	goal.fingers.finger3 = float(sys.argv[3])

    	client.send_goal(goal)

    	if not client.wait_for_result(rospy.Duration(5.0)):
        	print('        the gripper action timed-out')

    except rospy.ROSInterruptException:
        print "program interrupted before completion"
