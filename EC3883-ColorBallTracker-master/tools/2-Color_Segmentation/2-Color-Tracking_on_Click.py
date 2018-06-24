# import the necessary packages
from scipy import ndimage
import numpy as np
import cv2

# Tracked Colors.
tracked_colors = []


# Print on console the mouse position.
def onMouse (event, x, y, f, other):
    # Bring the image from the global context
    global shifted, tracked_colors

    # print("x:{}  y:{},  color = {}".format(x,y, image[y][x]))
    # Set thy callback event
    if (event == cv2.EVENT_LBUTTONDOWN):
        # Convert point to hSV
        hsv = cv2.cvtColor(shifted, cv2.COLOR_BGR2HSV)
        tracked_colors.append( hsv[y][x] )
        colorTracker(shifted, tracked_colors)



# Color Tracking function
def colorTracker(image, tracked_colors):
    # convert to HSV
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Calculate the HSV space of the color you're tracking.
    # if the list is not empty
    h_margin = 0.1 * 180
    s_margin = 0.3 * 255
    v_margin = 0.3 * 255

    if tracked_colors:
        # Iterate over the Colors
        for c in tracked_colors:
            # Calculate high and low margin for calculation.
            hsv_high = np.array([(c[0]+h_margin)%180, (c[1]+s_margin)%255, (c[2]+v_margin)%255])
            hsv_low = np.array([(c[0]-h_margin), (c[1]-s_margin), (c[2]-v_margin)])
            # Filter by color.
            mask = cv2.inRange(hsv, hsv_low, hsv_high)
            cv2.imshow("masked", mask)



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
