#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

colourMap = {
    'red' : [ [160, 128, 32], [180, 255, 255] ], #170
    'purple' : [ [120, 32, 32], [150, 255, 255] ], # 115
    'green' : [ [35, 64, 32], [70, 255, 255] ], #58
    'yellow' : [ [25, 96, 128], [45, 255, 255] ], # 34
    'blue' : [ [95, 150, 32], [110, 255, 255] ], # 104
    'orange' : [ [0, 160, 128], [20, 255, 255] ], # 9
}

## Hierarchy array constants
HIERARCHY_NEXT = 0
HIERARCHY_PREV = 1
HIERARCHY_FIRST_CHILD = 2
HIERARCHY_PARENT = 3

class block_detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( '/camera/rgb/image_rect_color', Image, self.imageCallback, queue_size=1 )
        self.image_pub = rospy.Publisher( 'annotated_image', Image, queue_size=1 )

    def imageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2( data, "bgr8" )
        except CvBridgeError, e:
            print e

        # Convert BGR to HSV
        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        coloursOfInterestMask = np.zeros( (hsvImage.shape[0], hsvImage.shape[1]), np.uint8 )

        ## Create a mask that matches any of the colours of interest
        drawContours = []
        for colourName, colourBounds in colourMap.iteritems():
            (colourMask, newContours) = self.processColour( hsvImage, colourName, colourBounds )
            coloursOfInterestMask = cv2.bitwise_or( coloursOfInterestMask, colourMask )
            drawContours.extend( newContours )

        # Bitwise-AND mask and original image
        res = cv2.bitwise_or( cv_image, np.ones( cv_image.shape, np.uint8 ), mask=coloursOfInterestMask )
            
        for cnt in drawContours:
            cv2.drawContours( res, drawContours, -1, (0,255,255), 2 )
            
        resultImage = self.bridge.cv2_to_imgmsg( res, encoding="passthrough" )
        
        self.image_pub.publish( resultImage )

    def processColour( self, hsvImage, colourName, colourBounds ):
        lowerBound = np.array( colourBounds[0] )
        upperBound = np.array( colourBounds[1] )
        colourMask = cv2.inRange( hsvImage, lowerBound, upperBound )
        colourContours, hierarchy = cv2.findContours( colourMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
        drawContours = []

        for index in range( 0, len( colourContours ) ):
            if self.isBlock( colourContours, hierarchy, index ):
                drawContours.append( colourContours[index] )

        return (colourMask, drawContours)

    def isBlock( self, contours, hierarchy, index ):
        contour = contours[index]
        ## Simple screen: throw out non-squares
        if not self.isSquare( contour ):
            return False

        childIndex = hierarchy[0][index][HIERARCHY_FIRST_CHILD]
        while ( childIndex >= 0 ):
            child = contours[childIndex]
            if ( self.isSquare( child ) ):
                childArea = cv2.contourArea( child )
                return True
            
            childIndex = hierarchy[0][childIndex][HIERARCHY_NEXT]
        
        return False

    def isSquare( self, contour ):
        approxShape = cv2.approxPolyDP( contour, 0.01 * cv2.arcLength( contour, True), True )
        area = cv2.contourArea( approxShape )
        return ( ( len( approxShape ) == 4 ) and ( area > 100 ) and ( cv2.isContourConvex( approxShape ) ) )


def main(args):
    rospy.init_node('block_detector')
    block_detector()
    rospy.loginfo( "Block detector initialised" )
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
