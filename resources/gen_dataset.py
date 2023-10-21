#!/usr/bin/env python3

# -- import module --
import cv2
import numpy as np
import random
import math


"""
This file generates the dataset for 4 different maze types.

Maze 0: 400x400 with no maze
Maze 1: 400x400 with T shaped maze
Maze 2: 400x400 with L shaped maze
Maze 3: 400x400 with X shaped maze
Maze 4: 400x400 with + shaped maze
"""
# ------------------------------------------------------------------------------
# -- Maze 0: 400x400 with no maze
# ------------------------------------------------------------------------------
# -- create a 400x400 image
img = np.zeros((400, 400, 3), np.uint8)
#  make all pixels in the image as white
img.fill(255)
# -- save the image
cv2.imwrite('maze0.png', img)

# ------------------------------------------------------------------------------
# -- Maze 1: 400x400 with T shaped maze
# ------------------------------------------------------------------------------
# -- create a 400x400 image
img = np.zeros((400, 400, 3), np.uint8)
#  make all pixels in the image as white
img.fill(255)

# -- draw the T shaped maze
""" Draw three rectangles to form a T shaped maze """
# -- draw the horizontal rectangle
cv2.rectangle(img, (0, 0), (400, 50), (0,0,0), -1)
# -- draw the left corner rectangle
cv2.rectangle(img, (0, 400), (150, 150), (0,0,0), -1)
# -- draw the right corner rectangle
cv2.rectangle(img, (400, 400), (400-150, 150), (0,0,0), -1)

# -- save the image
cv2.imwrite('maze1.png', img)

# ------------------------------------------------------------------------------
# -- Maze 2: 400x400 with L shaped maze
# ------------------------------------------------------------------------------
# -- create a 400x400 image
img = np.zeros((400, 400, 3), np.uint8)
#  make all pixels in the image as white
img.fill(255)

# -- draw the L shaped maze
""" Draw one rectangle to form a L shaped maze """
# -- draw the rectangle
cv2.rectangle(img, (150, 0), (400, 250), (0,0,0), -1)

# -- save the image
cv2.imwrite('maze2.png', img)

# ------------------------------------------------------------------------------
# -- Maze 3: 400x400 with X shaped maze
# ------------------------------------------------------------------------------
# -- create a 400x400 image
img = np.zeros((400, 400, 3), np.uint8)

# -- draw two lines of the X
cv2.line(img, (0,0), (400,400), (255, 255, 255), 100)
cv2.line(img, (400,0), (0,400), (255, 255, 255), 100)

# -- save the image
cv2.imwrite('maze3.png', img)

# ------------------------------------------------------------------------------
# -- Maze 4
# ------------------------------------------------------------------------------
# -- create a 400x400 image
img = np.zeros((400, 400, 3), np.uint8)

# -- draw the T shaped maze
""" Draw three rectangles to form a + shaped maze """
# -- draw the horizontal rectangle
cv2.rectangle(img, (0, 150), (400, 250), (255, 255, 255), -1)
# -- draw the vertical rectangle
cv2.rectangle(img, (150, 0), (250, 400), (255, 255, 255), -1)

# -- save the image
cv2.imwrite('maze4.png', img)