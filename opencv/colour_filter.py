#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from assign2.msg import HsvFilter
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class colour_filter:
    def __init__(self):
        self.bridge = CvBridge()
        self.lowerBound = np.array([0, 0, 0])
        self.upperBound = np.array([180, 255, 255])
        self.filter_sub = rospy.Subscriber('filter', HsvFilter, self.filterCallback, queue_size=1)
        self.image_sub = rospy.Subscriber( 'input_image', Image, self.imageCallback, queue_size=1 )
        self.image_pub = rospy.Publisher( 'filtered_image', Image, queue_size=1 )

    def filterCallback(self, data):
        self.lowerBound = np.array([data.hMin, data.sMin, data.vMin])
        self.upperBound = np.array([data.hMax, data.sMax, data.vMax])

    def imageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2( data, "bgr8" )
        except CvBridgeError, e:
            print e
            return

        # Convert BGR to HSV
        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        ## Create a mask that matches any of the colours of interest
        if self.upperBound[0] < self.lowerBound[0]:
            print(self.lowerBound, self.upperBound)
            lowerBound1 = np.array([self.lowerBound[0], self.lowerBound[1], self.lowerBound[2]])
            lowerBound2 = np.array([0, self.lowerBound[1], self.lowerBound[2]])
            upperBound1 = np.array([180, self.upperBound[1], self.upperBound[2]])
            upperBound2 = np.array([self.upperBound[0], self.upperBound[1], self.upperBound[2]])

            mask1 = cv2.inRange(hsvImage, lowerBound1, upperBound1)
            mask2 = cv2.inRange(hsvImage, lowerBound2, upperBound2)

            colourMask = cv2.bitwise_or(mask1, mask2)
        else:
            colourMask = cv2.inRange( hsvImage, self.lowerBound, self.upperBound )

        kernel = np.ones((2,2),np.uint8)
        colourMask = cv2.morphologyEx(colourMask, cv2.MORPH_DILATE, kernel)

        contours, hierarchy = cv2.findContours(colourMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        returnImage = cv2.cvtColor(colourMask, cv2.COLOR_GRAY2BGR)
        cv2.drawContours(returnImage, contours, -1, (0,0,255))

        resultImage = self.bridge.cv2_to_imgmsg( returnImage, encoding="passthrough" )
        self.image_pub.publish(resultImage)
        return

        # Bitwise-AND mask and original image
        res = cv2.bitwise_or( cv_image, np.ones( cv_image.shape, np.uint8 ), mask=colourMask )
            
        resultImage = self.bridge.cv2_to_imgmsg( res, encoding="passthrough" )
        
        self.image_pub.publish( resultImage )


def main(args):
    rospy.init_node('colour_filter')
    colour_filter()
    rospy.loginfo( "Colour filter initialised" )
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
