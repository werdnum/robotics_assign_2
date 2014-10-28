#!/usr/bin/env python

import os
import rospy
#import pickup_block
from assign2.srv import Pickup

def pickup_block_client():
	rospy.wait_for_service('pickup_block')
	try:
		pickup_block = rospy.ServiceProxy('pickup_block', Pickup)
		pickup_block(0.11, 1)
	except rospy.ServiceException, e:
        	print "Service call failed: %s"%e

if __name__ == '__main__':
	pickup_block_client()

#if __name__ == '__main__':
#	pickup_block.main(2,0)
#	pickup_block.main(0,1)
#	pickup_block.main(4,2)
#	pickup_block.main(8,3)
#	pickup_block.main(5,4)
#	pickup_block.main(1,5)
#	pickup_block.main(6,6)
#	pickup_block.main(7,7)
#	pickup_block.main(9,8)

