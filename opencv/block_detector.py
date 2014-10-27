#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from assign2.msg import Block
from cv_bridge import CvBridge, CvBridgeError

colourMap = {
    'red' : [ [170, 32, 0], [10, 255, 255] ],
    'orange' : [ [0, 160, 64], [15, 255, 255] ],
    'yellow' : [ [25, 128, 0], [30, 255, 255] ],
    'green' : [ [35, 64, 0], [45, 255, 255] ],
    'blue' : [ [90, 100, 0], [120, 200, 255] ],
    'purple' : [ [130, 64, 0], [160, 255, 255] ],
}

letterMap = {
    'green' : 'A',
    'blue' : 'B',
    'orange' : 'C',
    'purple' : 'D',
    'yellow' : 'E',
    'red' : 'F',
}

## Hierarchy array constants
HIERARCHY_NEXT = 0
HIERARCHY_PREV = 1
HIERARCHY_FIRST_CHILD = 2
HIERARCHY_PARENT = 3

class block_detector:
    def __init__(self, filename=None):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( 'image', Image, self.imageCallback, queue_size=1 )
        self.image_pub = rospy.Publisher( 'annotated_image', Image, queue_size=1 )
        self.block_pub = rospy.Publisher( 'blocks', Block, queue_size=26 )
        self.block_img_pub = rospy.Publisher( 'block_images', Image, queue_size=26 )

    def initKNN(self):
        print "Initialising letter classifier"
        self.kNN = cv2.KNearest()
        with np.load('knn.npz') as data:
            self.kNN.train(data['images'], data['letters'])
        print "Initialised letter classifier."

    def imageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2( data, "bgr8" )
        except CvBridgeError, e:
            print e

        # Convert BGR to HSV
        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        coloursOfInterestMask = np.zeros( (hsvImage.shape[0], hsvImage.shape[1]), np.uint8 )

        self.drawContours = []
        ## Create a mask that matches any of the colours of interest
        for colourName, colourBounds in colourMap.iteritems():
            colourMask = self.processColour( hsvImage, colourName, colourBounds )
            coloursOfInterestMask = cv2.bitwise_or( coloursOfInterestMask, colourMask )

        # Bitwise-AND mask and original image
        res = cv_image
        # res = cv2.bitwise_or( cv_image, np.ones( cv_image.shape, np.uint8 ), mask=coloursOfInterestMask )
         
        cv2.drawContours(res, self.drawContours, -1, (0, 0, 0))

        resultImage = self.bridge.cv2_to_imgmsg( res, encoding="bgr8" )

        self.image_pub.publish( resultImage )

    def processColour( self, hsvImage, colourName, colourBounds ):
        colourMask = self.getColourMask(hsvImage, colourBounds[0], colourBounds[1])
        kernel = np.ones((2,2),np.uint8)
        colourMask = cv2.morphologyEx(colourMask, cv2.MORPH_DILATE, kernel)
        colourContours, hierarchy = cv2.findContours( colourMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )

        for index in range( 0, len( colourContours ) ):
            self.drawContours.append(colourContours[index])
            if self.isBlock( colourContours, hierarchy, index ):
                contour = colourContours[index]
                cutImage = self.extractBlock(hsvImage, contour)
                cutImage = cv2.cvtColor(cutImage, cv2.COLOR_HSV2BGR)
                imgMsg = self.bridge.cv2_to_imgmsg(cutImage, encoding="bgr8")
                self.block_img_pub.publish(imgMsg)
                centre = self.getContourCentre(contour)
                blockMsg = Block()
                blockMsg.posX = float(centre[0]) / float(hsvImage.shape[1])
                blockMsg.posY = float(centre[1]) / float(hsvImage.shape[0])
                blockMsg.colour = colourName
                blockMsg.letter = self.getBlockLetter(cutImage)
                self.block_pub.publish(blockMsg)

        return colourMask

    def getColourMask(self, hsvImage, lowerBound, upperBound):
        lowerBound = np.array(lowerBound)
        upperBound = np.array(upperBound)
        if upperBound[0] < lowerBound[0]:
            lowerBound1 = np.array([lowerBound[0], lowerBound[1], lowerBound[2]])
            lowerBound2 = np.array([0, lowerBound[1], lowerBound[2]])
            upperBound1 = np.array([180, upperBound[1], upperBound[2]])
            upperBound2 = np.array([upperBound[0], upperBound[1], upperBound[2]])

            mask1 = cv2.inRange(hsvImage, lowerBound1, upperBound1)
            mask2 = cv2.inRange(hsvImage, lowerBound2, upperBound2)

            colourMask = cv2.bitwise_or(mask1, mask2)
        else:
            colourMask = cv2.inRange( hsvImage, lowerBound, upperBound )

        return colourMask

    def extractBlock(self, image, contour, width=100, height=None):
        if height == None:
            height = width
        approxSquare = self.getApproxSquare(contour)
        if len(approxSquare) != 4:
            print "Not a square; has %d points" % len(approxSquare)
            return
        srcPoints = np.empty((4,2), np.float32)
        centre = self.getContourCentre(approxSquare)
        for index in range(0, 4):
            pointX = approxSquare[index][0][0]
            pointY = approxSquare[index][0][1]
            point = (pointX, pointY)
            isLeft = pointX < centre[0]
            isTop = pointY < centre[1]

            if isLeft and isTop:
                srcPoints[0] = point
            elif isTop and not isLeft:
                srcPoints[1] = point
            elif not isLeft and not isTop:
                srcPoints[2] = point
            elif isLeft and not isTop:
                srcPoints[3] = point

        return self.extractPoints(image, srcPoints, width, height)

    def extractPoints(self, image, points, width, height):
        dstPoints = np.float32([[0,0], [width,0], [width,height], [0,height]])
        transform = cv2.getPerspectiveTransform(points, dstPoints)
        return cv2.warpPerspective(image, transform, (width, height))

    def isBlock( self, contours, hierarchy, index ):
        contour = contours[index]
        area = cv2.contourArea( contour )
        return (area > 7000) and (area < 11000)

    def getBlockLetter(self, blockImage):
        blockImage = cv2.cvtColor(blockImage, cv2.COLOR_BGR2GRAY)
        row = np.reshape(blockImage, -1)
        ret,result,neighbours,dist = self.kNN.find_nearest(testImages,k=3)
        return chr(result[0])

    def getApproxSquare( self, contour ):
        rect = cv2.minAreaRect(contour)
        points = cv2.cv.BoxPoints(rect)
        output = np.zeros(np.array([4,1,2]), np.float32)
        for i in range(0,4):
            output[i][0] = points[i]
        return output

    def getContourCentre(self, contour):
        moments = cv2.moments(contour)
        center_x = int(moments['m10'] / moments['m00'])
        center_y = int(moments['m01'] / moments['m00'])
        return (center_x, center_y)


def main(args):
    rospy.init_node('block_detector')
    block_detector()
    rospy.loginfo( "Block detector initialised" )
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
