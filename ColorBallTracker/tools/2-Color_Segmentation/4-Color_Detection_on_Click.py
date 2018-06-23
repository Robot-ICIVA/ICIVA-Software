# import the necessary packages
import numpy as np
import cv2
from collections import OrderedDict
import time

# Tracked Colors in BGR (NOT RGB).
# tracked_colors = [[103,94,230],[233,151,60]]
tracked_colors = []

# Dictionary of recognized color labels.
RGB_dictionary = OrderedDict({
        "RED": (255, 0, 0),
        "GREEN": (0, 255, 0),
        "BLUE": (0, 0, 255),
        "CYAN": (0, 255, 255),
        # It doesn't recognize Magenta very well.
        # "MAGENTA": (255, 0, 255),
        "YELLOW": (255, 255, 0),
    })


# Percentage margin for hue, saturation and value.
margins = { "hue": 0.05,
            "saturation": 0.2,
            "value": 0.2
            }


def color2Mask(image, tracked_colors, margins):

    # Unpack margins.
    marH = margins["hue"]
    marS = margins["saturation"]
    marV = margins["value"]

    # Transform the image to HSV
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # mask accumulator
    all_masks = np.zeros((image.shape[0], image.shape[1]), dtype = np.uint8)

    # Iterate over the list of tracked colors
    for color in tracked_colors:

        # Transform the color to hsv.
        hsv_color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2HSV)
        hsv_color = hsv_color[0][0]
        # print("hsv_color = {}".format(hsv_color))

        # Calculate the margins for the mask from the base colors and the margin
        low_margin  = [ int(hsv_color[0] - 180*marH), int(max(hsv_color[1] - 255*marS, 0)), int(max(hsv_color[2] - 255*marV, 0)) ]
        high_margin = [ int(hsv_color[0] + 180*marH), int(min(hsv_color[1] + 255*marS, 255)), int(min(hsv_color[2] + 255*marV, 255)) ]

        # If there is a Wrap Around on the Hue axis, Create 2 sets of margins.
        if (low_margin[0] < 0) or (high_margin[0] > 179):

            if (low_margin[0] < 0):
                # If the low margin is negative, do a margin set in the upper region of the color space
                low_margin1  = [ 179 - np.abs(low_margin[0]), low_margin[1], low_margin[2] ]
                high_margin1 = [ 179, high_margin[1], high_margin[2] ]
                # then, do a margin set in the lower region of the color space.
                low_margin2  = [ 0, low_margin[1], low_margin[2] ]
                high_margin2 = [ high_margin[0], high_margin[1], high_margin[2] ]

            if (high_margin[0] > 179):
                # If the low margin is negative, do a margin set in the upper region of the color space
                low_margin1  = [ low_margin[0], low_margin[1], low_margin[2] ]
                high_margin1 = [ 179, high_margin[1], high_margin[2] ]
                # then, do a margin set in the lower region of the color space.
                low_margin2  = [ 0, low_margin[1], low_margin[2] ]
                high_margin2 = [ high_margin[0] - 179, high_margin[1], high_margin[2] ]

            # Run both masks and append them together
            mask1 = cv2.inRange(hsv_image,np.array(low_margin1),np.array(high_margin1))
            mask2 = cv2.inRange(hsv_image,np.array(low_margin2),np.array(high_margin2))
            mask  = cv2.bitwise_or(mask1,mask2)

        # If There is no weird Hue overflow, then just calculate the normal mask.
        else:
            mask = cv2.inRange(hsv_image,np.array(low_margin),np.array(high_margin))

        # Append the generated masks onto a single variable
        all_masks  = cv2.bitwise_or(all_masks,mask)


    # Erosion and Dilation on the combined mask
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(5,5))
    all_masks = cv2.morphologyEx(all_masks, cv2.MORPH_OPEN, kernel)

    # Return the accumulated masks
    return all_masks


def mask2CentroidColor(image, mask, LAB_dictionary):
    # Calculate the contours of the image
    (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

    # initialize good contours
    tracked_cnts = []

    # loop over the contours
    for c in cnts:

        # Try to Eliminate wrong contours based on how much thier perimeter and area approximate an actual circle.
        area = cv2.contourArea(c)
        perimeter = cv2.arcLength(c,True)
        (x,y),radius = cv2.minEnclosingCircle(c)
        # Transform the center of the circle to pixels
        (x,y) = (int(x),int(y))
        circle_area = int( np.pi * radius**2)
        circle_perimeter = int( np.pi * radius*2)
        if not (circle_perimeter*0.9 < perimeter < circle_perimeter*1.1 and circle_area*0.6 < area < circle_area*1.4): continue

        ## If you reach this far, detect the contour color using LAB aproximation
        # Retrieve center color from the image
        color = image[y][x]
        # Transform color to LAB color space
        color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2LAB)
        # Calculate the vector difference between the selected color, and the known dictionary using numpy broadcasting.
        # Then compute the magnitue of the vector. Finally, retrieve the position of the value with the smallest difference.
        # To avoid overflowing, convert the arrays to float before substraction.
        min_idx = np.linalg.norm( LAB_dictionary["lab"].astype(np.float32) - color.astype(np.float32), axis=2 ).argmin()
        # Retrieve the label corresponding to that color.
        color_label = LAB_dictionary["name"][min_idx]

        ## Prepare array to return
        # Each ball is represented by an array  with the centroid, the radius, and the color label, like:
        # [ (x,y), radius, label ]
        tracked_cnts.append([(x,y), int(radius), color_label])

    # When the iteration through the contours has finished, return the list of tracked balls.
    return tracked_cnts




def RBG2LAB_labels(RGB_dictionary):
    """Convert an ordered dictionary of RGB colors to LAB format.
    returns a dictionary with a  list of converted values and a list of text labels. """

    RGB_dictionary
    lab_nparray = np.zeros((len(RGB_dictionary), 1, 3), dtype="uint8")
    color_names = []

    # loop over the colors dictionary
    for (i, (name, rgb)) in enumerate(RGB_dictionary.items()):
        # update the L*a*b* array and the color names list
        lab_nparray[i] = rgb
        color_names.append(name)

        # convert the L*a*b* array from the RGB color space
        # to L*a*b*
        lab = cv2.cvtColor(lab_nparray, cv2.COLOR_RGB2LAB)

    # Create a dictionary to easily pass the results around
    LAB_dictionary = {
            "lab": lab,
            "name":color_names
            }
    # Return the dictionary
    return LAB_dictionary

# Print on console the mouse position.
def onMouse (event, x, y, f, other):
    # Bring the image from the global context
    global shifted, tracked_colors

    # print("x:{}  y:{},  color = {}".format(x,y, image[y][x]))
    # Set thy callback event
    if (event == cv2.EVENT_LBUTTONDOWN):
        tracked_colors.append( shifted[y][x] )



# Create NamedWindow, and set callback.
cv2.namedWindow('Result')
cv2.setMouseCallback('Result', onMouse, 0 );


# Transform the recognized color to LAB color space.
LAB_dictionary = RBG2LAB_labels(RGB_dictionary)

# load the image and perform pyramid mean shift filtering
# to aid the thresholding step
image = cv2.imread("../Figures/Balls_2.jpg")
# Super filter the Image

while (1):
    image1 = image.copy()
    time_start = time.time()
    # shifted = cv2.pyrMeanShiftFiltering(image, 21, 51)
    # shifted = cv2.pyrMeanShiftFiltering(image, 10, 51)
    shifted = cv2.pyrMeanShiftFiltering(image, 5, 51)
    shifted = cv2.GaussianBlur(shifted, (9, 9), 0)
    time1 = time.time()
    ## Calculate the masks associated to the tracked colors
    mask = color2Mask(shifted, tracked_colors, margins)
    time2 = time.time()
    ## Calculate contours and colors of the tracked objects
    tracked_cnts = mask2CentroidColor(shifted, mask, LAB_dictionary)
    time3 = time.time()

    # Draw the amount of tracked balls.
    cv2.putText(image1, "Tracked Balls = {}".format(len(tracked_cnts)), (2, 22), cv2.FONT_HERSHEY_SIMPLEX,0.8, (255, 255, 255), 2)

    ## Draw the tracked contours on screen
    for cnt in tracked_cnts:
        # Draw a circle around each ball
        center = cnt[0]
        radius = cnt[1]
        color = RGB_dictionary[cnt[2]]
        cv2.circle(image1,center,radius,(color[2], color[1], color[0]),2)

        # Draw a white dot at the center of each ball
        cv2.circle(image1, center, 1, (255, 255, 255), -1)
    time4 = time.time()

    # Calculate speed of the algorithm
    cv2.putText(image1, "FPS = {}".format(round(1/(time4-time_start),1)), (2, image1.shape[0]-2), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)

    # Redraw the Image
    cv2.imshow("Result",  image1)

    #Print some informational results
    print("Mean Shift = {}".format(time1 - time_start))
    print("color2Mask = {}".format(time2 - time1))
    print("mask2CentroidColor = {}".format(time3 - time2))
    print("Draw contours = {}".format(time4 - time3))
    print("Total Time = {}\n".format(time4 - time_start))

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
