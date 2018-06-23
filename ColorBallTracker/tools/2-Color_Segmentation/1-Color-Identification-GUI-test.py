# import the necessary packages
from scipy import ndimage
import numpy as np
import cv2

# Print on console the mouse position.
def onMouse (event, x, y, f, other):
    global image
    print("x:{}  y:{},  color = {}".format(x,y, image[y][x]))



# load the image and perform pyramid mean shift filtering
# to aid the thresholding step
image = cv2.imread("../Figures/Balls_1.jpg")
# Super filter the Image
shifted = cv2.pyrMeanShiftFiltering(image, 10, 51)

# Create NamedWindow, and set callback
cv2.namedWindow('Shifted')
cv2.setMouseCallback('Shifted', onMouse, 0 );


cv2.imshow("Input", image)
cv2.imshow("Shifted", shifted)


cv2.waitKey(0)
cv2.destroyAllWindows()
