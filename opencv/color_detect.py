#!/usr/bin/env python

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

colourMap = {
    'red' : [ [160, 128, 32], [180, 255, 255] ], #170
    'purple' : [ [125, 32, 32], [145, 255, 255] ], # 115
    'green' : [ [45, 64, 32], [70, 255, 255] ], #58
    'yellow' : [ [25, 150, 32], [35, 255, 255] ], # 34
    'blue' : [ [95, 150, 32], [110, 255, 255] ], # 104
    'orange' : [ [0, 150, 32], [20, 255, 255] ], # 9
}

while(1):

    # Take each frame
    _, frame = cap.read()

    # Convert BGR to HSV
    hsvImage = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

    coloursOfInterestMask = np.zeros( (hsvImage.shape[0], hsvImage.shape[1]), np.uint8 )

    ## Create a mask that matches any of the colours of interest
    drawRects = []
    drawContours = []
    for colourName, colourBounds in colourMap.iteritems():
        lowerBound = np.array( colourBounds[0] )
        upperBound = np.array( colourBounds[1] )
        colourMask = cv2.inRange( hsvImage, lowerBound, upperBound )
        coloursOfInterestMask = cv2.bitwise_or( coloursOfInterestMask, colourMask )
        colourContours, hierarchy = cv2.findContours( colourMask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE )

        for cnt in colourContours:
            approx = cv2.approxPolyDP( cnt, 0.05 * cv2.arcLength( cnt, True), True )
            approxArea = cv2.contourArea( approx )
            
            if len(approx) == 4 and approxArea > 100 and cv2.isContourConvex( approx ):
                drawContours.append( approx )

    # Bitwise-AND mask and original image
    res = cv2.bitwise_or( frame, np.ones( frame.shape, np.uint8 ), mask=coloursOfInterestMask )
    for rect in drawRects:
        x, y, w, h = rect
        cv2.rectangle( res, (x, y), (x+w, y+h), (255,0, 0), 2 )
        
    for cnt in drawContours:
        cv2.drawContours( res, drawContours, -1, (0,255,255), 2 )

    cv2.imshow('res',res)
    k = cv2.waitKey(5) & 0xFF
    if k == 27:
        break

cv2.destroyAllWindows()

