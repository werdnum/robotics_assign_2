#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

colourMap = {
    'red' : [ [170, 32, 0], [10, 255, 255] ],
    'orange' : [ [0, 160, 64], [15, 255, 255] ],
    'yellow' : [ [25, 128, 0], [30, 255, 255] ],
    'green' : [ [35, 64, 0], [45, 255, 255] ],
    'blue' : [ [90, 100, 0], [120, 200, 255] ],
    'purple' : [ [130, 64, 0], [160, 255, 255] ],
}

workingMap = {
    'red' : [ [170, 128, 0], [180, 255, 255] ],
    'blue' : [ [90, 100, 0], [120, 200, 255] ],
    'green' : [ [60, 64, 0], [75, 255, 255] ],
    'yellow' : [ [25, 128, 0], [30, 255, 255] ],
}

blackLowerBound = np.array([0, 0, 0])
blackUpperBound = np.array([180, 255, 96])

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
        self.block_pub = rospy.Publisher( 'blocks', Image, queue_size=1 )
        self.image_counter = 0
        if filename != None:
            self.filename = filename
            print "Writing files to ", filename
        else:
            self.filename = None
        self.working_area = {'red' : None, 'yellow' : None, 'green' : None, 'blue' : None}

    def imageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2( data, "bgr8" )
        except CvBridgeError, e:
            print e

        # Convert BGR to HSV
        hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        ## Find corners
        boundingBox = self.getWorkingArea(cv_image)

        if not (None in boundingBox.values()):
            ordering = ['blue', 'red', 'yellow', 'green']
            transformPoints = [boundingBox[colour] for colour in ordering]
            transformPoints = np.array(transformPoints, np.float32)
            # for i in range(0,3):
            #     cv2.line(cv_image, (transformPoints[i][0], transformPoints[i][1]), \
            #         (transformPoints[i+1][0], transformPoints[i+1][1]), (255,255,0), 2)
            cv_image = self.extractPoints(cv_image, transformPoints, 1000, 600)
            hsvImage = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        else:
            return

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

    def getWorkingArea(self, image):
        # Find black boxes
        blackMaskedImage = cv2.inRange(image, blackLowerBound, blackUpperBound)
        blackContours, hierarchy = cv2.findContours(blackMaskedImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Set up mask for finding colour of a box
        centerMask = np.zeros(np.array([40,40]), np.uint8)
        centerMask[15:25, 15:25] = 255

        # Use the last stored working area as a starting point,
        # so if the detection is flaky then we have a starting point
        output = self.working_area
        for index in range(0, len(blackContours)):
            actualContour = blackContours[index]
            approx = self.getApproxSquare(actualContour)
            approxArea = cv2.contourArea(approx)
            actualArea = cv2.contourArea(actualContour)
            if approxArea < 1 or (actualArea / approxArea) < 0.2:
                continue
            if actualArea > 800:
                continue
            elif actualArea < 20:
                continue
            elif len(approx) != 4:
                continue
            else:
                center = self.getContourCentre(approx)

                squareImage = self.extractBlock(image, approx, 40)
                colour = cv2.mean(squareImage, mask=centerMask)
                colourName = self.classifyColour(colour)

                if colourName != None:
                    output[colourName] = center
        return output

    def classifyColour(self, colour):
        onePixelImage = np.array([[colour]], np.uint8)
        onePixelImage = cv2.cvtColor(onePixelImage, cv2.COLOR_BGR2HSV)
        hsvColor = onePixelImage[0,0]
        for colourName, colourBounds in workingMap.iteritems():
            match = True
            for itemIndex in range(0,len(colourBounds)):
                lowerBound = colourBounds[0]
                upperBound = colourBounds[1]
                if not (lowerBound[itemIndex] <= hsvColor[itemIndex] <= upperBound[itemIndex]):
                    match = False
                    break
            if match:
                break
        if not match:
            return None
        else:
            return colourName

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
                pubImage = self.bridge.cv2_to_imgmsg(cutImage, encoding="bgr8")
                self.block_pub.publish(pubImage)
                self.saveImage(cutImage, colourName)

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

    def saveImage(self, image, colourName):
        if (self.filename == None):
            return
        bgrImage = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
        self.image_counter = self.image_counter + 1
        filename = self.filename + "/" + \
            colourName + "/" + \
            str(self.image_counter) + ".png"
        print "Writing ", filename
        cv2.imwrite(filename, bgrImage)

    def isBlock( self, contours, hierarchy, index ):
        contour = contours[index]
        area = cv2.contourArea( contour )
        return (area > 7000) and (area < 11000)


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
    print args[1]
    if len(args) > 1 and not args[1].startswith('__name'):
        filename = args[1]
    else:
        filename = None
    rospy.init_node('block_detector')
    block_detector(filename)
    rospy.loginfo( "Block detector initialised" )
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
