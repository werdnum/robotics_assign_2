#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_saver:
    def __init__(self, filename=None):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( 'block_images', Image, self.imageCallback, queue_size=1 )
        self.image_counter = 0
        if filename != None:
            self.filename = filename
            print "Writing files to ", filename
        else:
            self.filename = None

    def imageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2( data, "bgr8" )
        except CvBridgeError, e:
            print e
            return

        if (self.filename == None):
            return
        self.image_counter = self.image_counter + 1
        filename = self.filename + "/" + \
            str(self.image_counter) + ".png"
        print "Writing ", filename
        cv2.imwrite(filename, cv_image)


def main(args):
    if len(args) > 1 and not args[1].startswith('__name'):
        filename = args[1]
    else:
        filename = None
    rospy.init_node('image_saver')
    image_saver(filename)
    rospy.loginfo( "Image saver: saving to %s" % filename )
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
