#!/usr/bin/env python

import cv2
import numpy as np

cap = cv2.VideoCapture(0)

while (1):
    _, frame = cap.read()

    cv2.imshow( "Capture", frame )
    k = cv2.waitKey( 5 ) & 0xFF
    if ( k == 27 ):
        break

cv2.destroyAllWindows()

