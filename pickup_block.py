#! /usr/bin/env python

import sys
#import cartesian_move
#import finger_move
import math
import rospy
import os

def main(pos1, pos2):
	#rospy.init_node('jaco_pickup_block')
	xpos_pick = 0.26 - int(pos1) * 0.047
	xpos_pick = str(xpos_pick)
	#cartesian_move(xpos_pick, -0.6, 0.1, 0.707, 0.707, 0, 0)
	os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.1 0.707 0.707 0 0') # Move over block
	#finger_move(1.25, 1.25, 1.25)
	os.system('python finger_move.py 1.25 1.25 1.25') # Open fingers
	#cartesian_move(xpos_pick, -0.6, 0.03, 0.707, 0.707, 0, 0)	
	os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.03 0.707 0.707 0 0')
	#finger_move(36, 36, 1.25)
	os.system('python finger_move.py 36 36 1.25') # Grab block
	#cartesian_move(xpos_pick, -0.6, 0.1, 0.707, 0.707, 0, 0)
	os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.1 0.707 0.707 0 0')
	xpos_place = 0.26 - int(pos2) * 0.06
	zpos_place = 0 - int(pos2) * 0.0001
	xpos_place = str(xpos_place)
	zpos_place = str(zpos_place)
	#cartesian_move(xpos_place, -0.46, 0.1, 0.707, 0.707, 0, 0)
	os.system('python cartesian_move.py ' + xpos_place + ' -0.46 0.1 0.707 0.707 0 0') # Move to new pos
	#cartesian_move(xpos_place, -0.46, zpos_place, 0.707, 0.707, 0, 0)
	os.system('python cartesian_move.py ' + xpos_place + ' -0.46 ' + zpos_place + ' 0.707 0.707 0 0')
	#finger_move(1.25, 1.25, 1.25)
	os.system('python finger_move.py 1.25 1.25 1.25') # Open fingers
	#cartesian_move(xpos_place, -0.46, 0.1, 0.707, 0.707, 0, 0)
	os.system('python cartesian_move.py ' + xpos_place + ' -0.46 0.1 0.707 0.707 0 0')

if __name__ == '__main__':
	main(sys.argv[1], sys.argv[2])
	
