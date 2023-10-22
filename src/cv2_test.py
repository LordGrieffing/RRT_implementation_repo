#!/usr/bin/python3

import cv2
import numpy as np

# -- import an image and convert it to a binary image
img = cv2.imread('maze1.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
#img[300, 300] = [255, 255, 255]


img[350, 200] = [0, 0, 255]
cv2.imshow('My Image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()