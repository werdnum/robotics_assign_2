#!/usr/bin/env python

import sys
#import cartesian_move
#import finger_move
import math
import rospy
import os

from assign2.srv import Pickup, Finger, Cartesian

def handle_pickup_block(req):
	print "Received request to move block from ", req.pos1, " to ", req.pos2
	#rospy.init_node('jaco_pickup_block')
	#xpos_pick = 0.26 - int(pos1) * 0.047
	# For pos1 as percentage of distance along top
	xpos_pick = 0.32 - req.pos1 * 0.58
	#xpos_pick = str(xpos_pick)
	#os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.1 0.707 0.707 0 0') # Move over block
	rospy.wait_for_service('cartesian_move')
	try:
		cartesian_move = rospy.ServiceProxy('cartesian_move', Cartesian)
		cartesian_move(float(xpos_pick), -0.6, 0.1, 0.707, 0.707, 0.0, 0.0)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
	#os.system('python finger_move.py 1.25 1.25 1.25') # Open fingers
	rospy.wait_for_service('finger_move')
	try:
		finger_move = rospy.ServiceProxy('finger_move', Finger)
		finger_move(1.25, 1.25, 1.25)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
	rospy.wait_for_service('cartesian_move')
	try:
		cartesian_move = rospy.ServiceProxy('cartesian_move', Cartesian)
		cartesian_move(float(xpos_pick), -0.6, 0.03, 0.707, 0.707, 0.0, 0.0)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
	#os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.03 0.707 0.707 0 0')
	rospy.wait_for_service('finger_move')
	try:
		finger_move = rospy.ServiceProxy('finger_move', Finger)
		finger_move(36, 36, 1.25)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e	
	#os.system('python finger_move.py 36 36 1.25') # Grab block
	#os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.1 0.707 0.707 0 0')
	xpos_place = 0.26 - req.pos2 * 0.06
	zpos_place = 0 - req.pos2 * 0.0001
	#xpos_place = str(xpos_place)
	#zpos_place = str(zpos_place)
	rospy.wait_for_service('cartesian_move')
	try:
		cartesian_move = rospy.ServiceProxy('cartesian_move', Cartesian)
		cartesian_move(float(xpos_pick), -0.6, 0.1, 0.707, 0.707, 0.0, 0.0)
		cartesian_move(float(xpos_place), -0.46, 0.1, 0.707, 0.707, 0.0, 0.0)
		cartesian_move(float(xpos_place), -0.46, float(zpos_place), 0.707, 0.707, 0.0, 0.0)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
	#os.system('python cartesian_move.py ' + xpos_place + ' -0.46 0.1 0.707 0.707 0 0') # Move to new pos
	#os.system('python cartesian_move.py ' + xpos_place + ' -0.46 ' + zpos_place + ' 0.707 0.707 0 0')
	rospy.wait_for_service('finger_move')
	try:
		finger_move = rospy.ServiceProxy('finger_move', Finger)
		finger_move(1.25, 1.25, 1.25)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
	#os.system('python finger_move.py 1.25 1.25 1.25') # Open fingers
	rospy.wait_for_service('cartesian_move')
	try:
		cartesian_move = rospy.ServiceProxy('cartesian_move', Cartesian)
		cartesian_move(float(xpos_place), -0.46, 0.1, 0.707, 0.707, 0.0, 0.0)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e
	#os.system('python cartesian_move.py ' + xpos_place + ' -0.46 0.1 0.707 0.707 0 0')
	return True

def pickup_block_server():
	rospy.init_node('jaco_pickup_block')
	s = rospy.Service('pickup_block', Pickup, handle_pickup_block)
	rospy.spin()

if __name__ == '__main__':
	pickup_block_server()

#if __name__ == '__main__':
#	main(sys.argv[1], sys.argv[2])
