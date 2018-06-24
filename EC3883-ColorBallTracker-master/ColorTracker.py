# import the necessary packages
import numpy as np
import cv2
from collections import OrderedDict
import time
import argparse

class ColorTracker:
    """ Class defined to track the colors of the balls in the image """


    def __init__(self, tracked_colors = [], debug_flag = False):
        ## Just initialice some global variables.
        self.tracked_colors = tracked_colors

        # Initialize debugging flag
        self.debug_flag = debug_flag

        # Dictionary of recognized color labels.
        self.RGB_dictionary = OrderedDict({
            "RED": (255, 0, 0),
            "GREEN": (0, 255, 0),
            "BLUE": (0, 0, 255),
            "CYAN": (0, 255, 255),
            # It doesn't recognize Magenta very well.
            # "MAGENTA": (255, 0, 255),
            "YELLOW": (255, 255, 0),
        })

        # Minimum sizes in pixels of the balls detected
        self.ball_min_size = 100 #pixels


        # Percentage margin for hue, saturation and value.
        self.margins = { "hue": 0.05,
                        "saturation": 0.3,
                        "value": 0.3
                        }
        # self.margins = { "hue": 0.05,
        #                 "saturation": 0.2,
        #                 "value": 0.2
        #                 }

        # Transform the RGB dictionary to LAB.
        self.LAB_dictionary = self.RBG2LAB_labels(self.RGB_dictionary)

        # Place holder for the shifted image.
        self.shifted = None



    def color2Mask(self, image):

        """ Calculate a Mask composed of all the Balls that where found in the image.
            The colors used for tracking are those found in self.tracked_colors.
        """

        # Unpack margins.
        marH = self.margins["hue"]
        marS = self.margins["saturation"]
        marV = self.margins["value"]

        # Transform the image to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # mask accumulator
        all_masks = np.zeros((image.shape[0], image.shape[1]), dtype = np.uint8)

        # Iterate over the list of tracked colors
        for color in self.tracked_colors:

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



    def mask2CentroidColor(self, image, mask):

        """ Take the mask generated from color2Mask() and use contour analysis to try to detect and locate the balls.
        """

        # Calculate the contours of the image
        (_, cnts, _) = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL,cv2.CHAIN_APPROX_SIMPLE)

        # initialize good contours
        tracked_cnts = []

        # loop over the contours
        for c in cnts:

            # Try to Eliminate wrong contours based on how much thier perimeter and area approximate an actual circle.
            area = cv2.contourArea(c)
            # Filter contours by area
            if area < self.ball_min_size: continue

            perimeter = cv2.arcLength(c,True)
            (x,y),radius = cv2.minEnclosingCircle(c)
            # Transform the center of the circle to pixels
            (x,y) = (int(x),int(y))
            # circle_area = int( np.pi * radius**2)
            # circle_perimeter = int( np.pi * radius*2)
            # if not (circle_perimeter*0.5 < perimeter < circle_perimeter*1.5 and circle_area*0.4 < area < circle_area*1.6): continue

            # If the contour can be approximated by less than 8 lines, then it probably isn't a circle.
            peri = cv2.arcLength(c, True)
            approx = cv2.approxPolyDP(c, 0.04 * peri, True)
            if len(approx) <= 4: continue

            ## If you reach this far, detect the contour color using LAB aproximation
            # Retrieve center color from the image
            color = image[y][x]
            # Transform color to LAB color space
            color = cv2.cvtColor(np.uint8([[color]]), cv2.COLOR_BGR2LAB)
            # Calculate the vector difference between the selected color, and the known dictionary using numpy broadcasting.
            # Then compute the magnitue of the vector. Finally, retrieve the position of the value with the smallest difference.
            # To avoid overflowing, convert the arrays to float before substraction.
            min_idx = np.linalg.norm( self.LAB_dictionary["lab"].astype(np.float32) - color.astype(np.float32), axis=2 ).argmin()
            # Retrieve the label corresponding to that color.
            color_label = self.LAB_dictionary["name"][min_idx]

            ## Prepare array to return
            # Each ball is represented by an array  with the centroid, the radius, and the color label, like:
            # [ (x,y), radius, label ]
            tracked_cnts.append([(x,y), int(radius), color_label])

        # When the iteration through the contours has finished, return the list of tracked balls.
        return tracked_cnts



    def RBG2LAB_labels(self, RGB_dictionary):
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



    def run(self, image):
        """Convert an ordered dictionary of RGB colors to LAB format.
        returns a dictionary with a  list of converted values and a list of text labels. """

        shifted = cv2.pyrMeanShiftFiltering(image, 5, 51)
        shifted = cv2.GaussianBlur(shifted, (9, 9), 0)
        # Save the preprocessed image for further  use
        self.shifted = shifted
        ## Calculate the masks associated to the tracked colors
        mask = self.color2Mask(shifted)
        ## Calculate contours and colors of the tracked objects
        tracked_balls = self.mask2CentroidColor(shifted, mask)
        # Return the dictionary

        # Print the Mask if we are in debug mode
        if self.debug_flag: cv2.imshow("mask",  mask)

        return tracked_balls



# CV Callback
def onMouse (event, x, y, f, other):
    # Bring the image from the global context
    global tracker

    # print("x:{}  y:{},  color = {}".format(x,y, image[y][x]))
    # Set thy callback event
    if (event == cv2.EVENT_LBUTTONDOWN):
        tracker.tracked_colors.append( tracker.shifted[y][x] )


def main():

    ap = argparse.ArgumentParser()

    group = ap.add_mutually_exclusive_group()
    group.add_argument("-c", "--camera", type=int, default=0, help="Index of the camera to use")
    group.add_argument("-i", "--image",  type=str, help="Static image to run the algorithm")
    ap.add_argument("-d", "--debug",  action="store_true", default=False, help="Print debbuging images")
    args = vars(ap.parse_args())

    # Create NamedWindow, and set callback.
    cv2.namedWindow('Result')
    cv2.setMouseCallback('Result', onMouse, 0 );

    global tracker
    # Initialize color tracking object
    tracker = ColorTracker(debug_flag = args["debug"])


    # Start Media Input
    if args["image"] == None:
        # Initialize camera input
        cap = cv2.VideoCapture(args["camera"])
    else:
        # Load image from Disk
        image = cv2.imread(args["image"])
        if (image is None):
            print("Error: Image '{}' not Found".format(args["image"]))



    while (1):

        if args["image"] == None:
            # Capture frame-by-frame
            ret, image = cap.read()
        else:
            image_copy = image.copy()


        # Create a Copy of the image.
        image_copy = image.copy()

        # Run the Color Tracker
        start_time = time.time()
        tracked_balls = tracker.run(image.copy())
        end_time = time.time()

        # Draw the amount of tracked balls.
        cv2.putText(image_copy, "Tracked Balls = {}".format(len(tracked_balls)), (2, 22), cv2.FONT_HERSHEY_SIMPLEX,0.8, (255, 255, 255), 2)

        ## Draw the tracked contours on screen
        for ball in tracked_balls:
            # Draw a circle around each ball
            center = ball[0]
            radius = ball[1]
            color = tracker.RGB_dictionary[ball[2]]
            cv2.circle(image_copy,center,radius,(color[2], color[1], color[0]),2)

            # Draw a white dot at the center of each ball
            cv2.circle(image_copy, center, 1, (255, 255, 255), -1)

        # Calculate speed of the algorithm
        cv2.putText(image_copy, "FPS = {}".format(round(1/(end_time-start_time),1)), (2, image_copy.shape[0]-2), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)

        # Redraw the Image
        cv2.imshow("Result",  image_copy)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break


    # CLose everything when done
    if args["image"] == None: cap.release()
    cv2.destroyAllWindows()



if __name__ == '__main__':
    main()
