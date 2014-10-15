#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

colourMap = {
    'red' : [ [170, 128, 0], [180, 255, 255] ],
    'orange' : [ [0, 160, 64], [15, 255, 255] ],
    'yellow' : [ [25, 128, 0], [30, 255, 255] ],
    'green' : [ [35, 64, 0], [45, 255, 255] ],
    'blue' : [ [90, 100, 0], [120, 200, 255] ],
    'purple' : [ [130, 64, 0], [160, 255, 255] ],
}

## Hierarchy array constants
HIERARCHY_NEXT = 0
HIERARCHY_PREV = 1
HIERARCHY_FIRST_CHILD = 2
HIERARCHY_PARENT = 3

class block_detector:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( '/image', Image, self.imageCallback, queue_size=1 )
        self.image_pub = rospy.Publisher( 'annotated_image', Image, queue_size=1 )
        self.block_pub = rospy.Publisher( 'blocks', Image, queue_size=1 )

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
            length = len(cnt)
            colour = (128,128,128)
            if length == 3:
                colour = (255, 0, 0)
            elif length == 4:
                colour = (0, 255, 0)
            elif length == 5:
                colour = (0, 0, 255)
            elif length > 5:
                colour = (255, 255, 255)
            cv2.drawContours( res, [cnt], -1, colour, 1 )

        resultImage = self.bridge.cv2_to_imgmsg( res, encoding="passthrough" )

        self.image_pub.publish( resultImage )

    def processColour( self, hsvImage, colourName, colourBounds ):
        lowerBound = np.array( colourBounds[0] )
        upperBound = np.array( colourBounds[1] )
        colourMask = cv2.inRange( hsvImage, lowerBound, upperBound )
        kernel = np.ones((2,2),np.uint8)
        colourMask = cv2.morphologyEx(colourMask, cv2.MORPH_CLOSE, kernel)
        colourContours, hierarchy = cv2.findContours( colourMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )
        drawContours = []
        otherPoints = np.float32([[0,0], [100,0], [100,100], [0,100]])

        for index in range( 0, len( colourContours ) ):
            if self.isBlock( colourContours, hierarchy, index ):
                contour = colourContours[index]
                approxSquare = self.getApproxSquare(contour)
                transform = cv2.getAffineTransform(approxSquare, otherPoints)
                cutImage = cv2.warpAffine(hsvImage, transform, (100, 100) )
                pubImage = self.bridge.cv2_to_imgmsg(cutImage, encoding="passthrough")
                self.block_pub.publish(pubImage)

        return (colourMask, drawContours)

    def isBlock( self, contours, hierarchy, index ):
        contour = contours[index]
        ## Simple screen: throw out non-squares
        if not self.isSquare( contour ):
            return False
        else:
            return True

        childIndex = hierarchy[0][index][HIERARCHY_FIRST_CHILD]
        while ( childIndex >= 0 ):
            child = contours[childIndex]
            if ( self.isSquare( child ) ):
                childArea = cv2.contourArea( child )
                return True
            
            childIndex = hierarchy[0][childIndex][HIERARCHY_NEXT]
        
        return False

    def isSquare( self, contour ):
        approxShape = self.getApproxSquare(contour)
        rectArea = cv2.contourArea( approxShape )
        actualArea = cv2.contourArea( contour )
        return (len( approxShape ) == 4) and \
            (actualArea > 100)


    def getApproxSquare( self, contour ):
        convexHull = cv2.convexHull(contour)
        return cv2.approxPolyDP( convexHull, 0.125 * cv2.arcLength( contour, True), True )


def main(args):
    rospy.init_node('block_detector')
    block_detector()
    rospy.loginfo( "Block detector initialised" )
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
