import numpy as np
import cv2
import cv2.aruco as aruco


'''
    drawMarker(...)
        drawMarker(dictionary, id, sidePixels[, img[, borderBits]]) -> img
'''
# second parameter is id number
# last parameter is total image size

# Generate marker.
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# print marker 27
i = 27
img = aruco.drawMarker(aruco_dict, i, 600)
cv2.imwrite("marker_{}.jpg".format(i), img)

# Sohw the Image
cv2.imshow('frame',img)
cv2.waitKey(0)
cv2.destroyAllWindows()
