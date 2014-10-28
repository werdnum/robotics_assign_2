#!/usr/bin/env python

import roslib; roslib.load_manifest('jaco_demo')
import rospy

import sys

import actionlib
import jaco_msgs.msg

from assign2.srv import Finger

def handle_finger_move(req):
    try:
       	#rospy.init_node('jaco_finger_move')

       	action_address = '/jaco_arm_driver/fingers/finger_positions'
    	client = actionlib.SimpleActionClient(action_address,
                                          jaco_msgs.msg.SetFingersPositionAction)

    	client.wait_for_server()

    	goal = jaco_msgs.msg.SetFingersPositionGoal()
    	goal.fingers.finger1 = req.pos1
    	goal.fingers.finger2 = req.pos2
	goal.fingers.finger3 = req.pos3

    	client.send_goal(goal)

    	if not client.wait_for_result(rospy.Duration(5.0)):
        	print('        the gripper action timed-out')

	return True

    except rospy.ROSInterruptException:
        print "program interrupted before completion"

def finger_move_server():
	rospy.init_node('jaco_finger_move')
	s = rospy.Service('finger_move', Finger, handle_finger_move)
	rospy.spin();

if __name__ == '__main__':
	finger_move_server()

#if __name__ == '__main__':
#	main(sys.argv[1], sys.argv[2], sys.argv[3])
