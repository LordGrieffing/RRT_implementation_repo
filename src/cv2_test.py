#!/usr/bin/python3

import cv2
import numpy as np

# -- import an image and convert it to a binary image
img = cv2.imread('maze0.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
#img[300, 300] = [255, 255, 255]


img[100, 50] = [0, 0, 0]
img[80, 150] = [0, 0, 0]

def bresenham_line(x1, y1, x2, y2):
    # Setup initial conditions
    points = []
    dx = abs(x2 - x1)
    dy = abs(y2 - y1)
    x, y = x1, y1
    sx = 1 if x1 < x2 else -1
    sy = 1 if y1 < y2 else -1

    # Decision variables for determining the next point
    if dx > dy:
        err = dx / 2
        while x != x2:
            points.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2
        while y != y2:
            points.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy

    points.append((x, y))  # Add the final point
    return points



line = bresenham_line(50, 100, 150, 80)
for i in range(len(line)):
    img[line[i][1], line[i][0]] = [0, 0 ,0]



cv2.imshow('My Image',img)
cv2.waitKey(0)
cv2.destroyAllWindows()