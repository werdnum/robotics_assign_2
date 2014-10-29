#!/usr/bin/env python

import sys
import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

workingMap = {
    'red' : [ [170, 128, 64], [180, 255, 255] ],
    'blue' : [ [90, 100, 64], [120, 200, 255] ],
    'green' : [ [60, 64, 64], [75, 255, 255] ],
    'yellow' : [ [25, 128, 64], [30, 255, 255] ],
}

blackLowerBound = np.array([0, 0, 0])
blackUpperBound = np.array([180, 96, 64])

## Hierarchy array constants
HIERARCHY_NEXT = 0
HIERARCHY_PREV = 1
HIERARCHY_FIRST_CHILD = 2
HIERARCHY_PARENT = 3

class working_extractor:
    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber( 'image', Image, self.imageCallback, queue_size=1 )
        self.image_pub = rospy.Publisher( 'working_area', Image, queue_size=1 )
	self.image_debug_pub = rospy.Publisher( 'working_area_debug', Image, queue_size=1 )
        self.working_area = {colour: None for colour in workingMap.keys()}
        self.real_colours = {colour: None for colour in workingMap.keys()}
        self.colour_real_contours = {colour: None for colour in workingMap.keys()}
        self.colour_squares = {colour: None for colour in workingMap.keys()}

    def imageCallback(self,data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2( data, "bgr8" )
        except CvBridgeError, e:
            print e

	print "Got image"

        ## Find corners
        boundingBox = self.getWorkingArea(cv_image)
	debug_image = cv_image.copy()

        if not (None in boundingBox.values()):
            ordering = ['blue', 'red', 'yellow', 'green']
            transformPoints = [boundingBox[colour] for colour in ordering]
            transformPoints = np.array(transformPoints, np.float32)
            for i in range(0,3):
                cv2.line(debug_image, (transformPoints[i][0], transformPoints[i][1]), \
                    (transformPoints[i+1][0], transformPoints[i+1][1]), (255,255,0), 2)
            cv_image = self.extractPoints(cv_image, transformPoints, 1000, 600)
        
        for colour, pos in boundingBox.iteritems():
            cv2.circle(debug_image, pos, 20, self.real_colours[colour], -1)

	debugImage = self.bridge.cv2_to_imgmsg(debug_image, encoding="passthrough")
        self.image_debug_pub.publish(debugImage)

        resultImage = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgr8")
        self.image_pub.publish(resultImage)

    def getWorkingArea(self, image): # Image in BGR
        # Find black boxes
	hsvImage = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        blackMaskedImage = cv2.inRange(hsvImage, blackLowerBound, blackUpperBound)

	# debugImage = self.bridge.cv2_to_imgmsg(blackMaskedImage, encoding="passthrough")
        # self.image_debug_pub.publish(debugImage)

        blackContours, hierarchy = cv2.findContours(blackMaskedImage, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

        # Set up mask for finding colour of a box
        centerMask = np.zeros(np.array([40,40]), np.uint8)
        centerMask[15:25, 15:25] = 255

        # Use the last stored working area as a starting point,
        # so if the detection is flaky then we have a starting point
        output = {colour: None for colour in workingMap.keys()}
        biggest_box = {colour: 0 for colour in workingMap.keys()}
        for index in range(0, len(blackContours)):
            actualContour = blackContours[index]
            approx = self.getApproxSquare(actualContour)
            approxArea = cv2.contourArea(approx)
            actualArea = cv2.contourArea(actualContour)
            if approxArea < 1 or (actualArea / approxArea) < 0.7:
                continue
            elif actualArea < 100:
                continue
            elif len(approx) != 4:
                continue
            else:
                center = self.getContourCentre(approx)

                squareImage = self.extractBlock(image, approx, 40)
                colour = cv2.mean(squareImage, mask=centerMask)
		print "Found black box at ", center[0], ", ", center[1], " with colour ", colour
                colourName = self.classifyColour(colour)

                if colourName != None and actualArea > biggest_box[colourName]:
                    output[colourName] = center
                    self.real_colours[colourName] = colour
                    biggest_box[colourName] = actualArea
        return output

    def classifyColour(self, colour): # BGR colour
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
    rospy.init_node('working_extractor')
    working_extractor()
    rospy.loginfo( "Working area extractor" )
    rospy.spin()

if __name__ == '__main__':
    main(sys.argv)
