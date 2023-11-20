#!/usr/bin/python3

import cv2
import numpy as np

# -- import an image and convert it to a binary image
img = cv2.imread('map_sequence_2.png')
gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
ret, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
img[420, 270] = [0, 0, 255]
end = [285, 300]
inflateVal = 25



def erode_image(map_array, filter_size = 9):
    """ Erodes the image to reduce the chances of robot colliding with the wall
    each pixel is 0.05 meter. The robot is 30 cm wide, that is 0.3 m. Half is
    0.15 m. If we increase the walls by 20 cm on either side, the kernel should
    be 40 cm wide. 0.4 / 0.05 = 8
    """
    kernel = np.ones((filter_size,filter_size), np.uint8)
    eroded_img = cv2.erode(map_array, kernel, iterations = 1)
    return eroded_img


map = erode_image(img)

cv2.line(map, ((end[0] - inflateVal), (end[1] + inflateVal)), ((end[0] + inflateVal), (end[1] + inflateVal)), (255, 0, 0), 1)
cv2.line(map, ((end[0] - inflateVal), (end[1] - inflateVal)), ((end[0] + inflateVal), (end[1] - inflateVal)), (255, 0, 0), 1)
cv2.line(map, ((end[0] - inflateVal), (end[1] + inflateVal)), ((end[0] - inflateVal), (end[1] - inflateVal)), (255, 0, 0), 1)
cv2.line(map, ((end[0] + inflateVal), (end[1] - inflateVal)), ((end[0] + inflateVal), (end[1] + inflateVal)), (255, 0, 0), 1)



cv2.imshow('My Image',map)
cv2.waitKey(0)
cv2.destroyAllWindows()



# map_edit_sequence_1.png start = [300, 190]
# map_sequence_5.png end = [350, 250]
# map_sequence_4.png end = [320, 250]
# map_sequence_3.png end = [280, 240]
# map_sequence_2.png end = [280, 300]