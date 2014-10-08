#! /usr/bin/env python

import sys
import os
import math

if __name__ == '__main__':

	xpos_pick = 0.26 - int(sys.argv[1]) * 0.0475
	xpos_pick = str(xpos_pick)
	os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.1 0.707 0.707 0 0') # Move over block
	os.system('python finger_move.py 1.25 1.25 1.25') # Open fingers
	os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.03 0.707 0.707 0 0')
	os.system('python finger_move.py 36 36 1.25') # Grab block
	os.system('python cartesian_move.py ' + xpos_pick + ' -0.6 0.1 0.707 0.707 0 0')
	xpos_place = 0.26 - int(sys.argv[2]) * 0.056
	zpos_place = 0 - int(sys.argv[2]) * 0.0015
	xpos_place = str(xpos_place)
	zpos_place = str(zpos_place)
	os.system('python cartesian_move.py ' + xpos_place + ' -0.46 0.1 0.707 0.707 0 0') # Move to new pos
	os.system('python cartesian_move.py ' + xpos_place + ' -0.46 ' + zpos_place + ' 0.707 0.707 0 0')
	os.system('python finger_move.py 1.25 1.25 1.25') # Open fingers
	os.system('python cartesian_move.py ' + xpos_place + ' -0.46 0.1 0.707 0.707 0 0')
	
