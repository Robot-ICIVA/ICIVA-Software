import numpy as np
import cv2
import cv2.aruco as aruco
import argparse



class ARTracker:

    def __init__(self, marker_size = 0.144, field_length = 0.46, field_width = 0.46, debug_flag = False):

        # Our operations on the frame come here
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters =  aruco.DetectorParameters_create()
        # Activate subpixel corner refinement.
        self.parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
        self.parameters.cornerRefinementMinAccuracy = 0.1
        # Initialize M
        self.M = np.array([[1.0, 0.0, 0.0], [0.0, 1.0, 0.0], [0.0, 0.0, 1.0]])

        # Size of the side of the AR marker in meters.
        self.marker_size = marker_size

        # Size in meters of the width of the perspective shifted floor.
        self.field_width = field_width

        # Size in meters of the length of the perspective shifted floor.
        self.field_length = field_length

        # Flag that defines whether to recalculate the Perspective transformation (True) or to reuse the last one (False)
        self.Compute_M = True

        # Place holder for the ratio of pixels to meters and the pixel size of the warped image.
        self.pixel_to_meters = 0
        self.pixel_width = 0
        self.pixel_length = 0

        # Last Marker position
        self.marker_corners = []

        # Initialize debug mode.
        self.debug_flag = debug_flag

    def four_point_transform(self, image, pts):

        # Check whether to recalculate the homogeneous transformation matrix M
        # or if to just use the old one.

        if self.Compute_M:
            # Calculate the size of each of the sides of the square,
            # and use the maximum for the reference size of the warped
            # image.
            side1 = np.linalg.norm(pts[0]-pts[1])
            side2 = np.linalg.norm(pts[1]-pts[2])
            side3 = np.linalg.norm(pts[2]-pts[3])
            side4 = np.linalg.norm(pts[3]-pts[0])

            side = max(side1, side2, side3, side4)


            # Compute pixel to distance ratio.
            self.pixel_to_meters = self.marker_size / side

            # Compute pixel size of the resulting Image.
            self.pixel_length = int(self.field_length / self.pixel_to_meters)
            self.pixel_width  = int(self.field_width / self.pixel_to_meters)

            # Calculate the destination points for the transformation.
            dst = np.array([
            [0, self.pixel_length],
            [0,self.pixel_length - (side-1)],
            [side - 1, self.pixel_length - (side-1)],
            [side - 1, self.pixel_length]], dtype = "float32")

            # Compute the perspective transform matrix and then apply it
            self.M = cv2.getPerspectiveTransform(pts, dst)




    def run(self, image):

        self.marker_corners, ids, rejectedImgPoints = aruco.detectMarkers(image, self.aruco_dict, parameters=self.parameters)

        # Draw the found markers in the image, and show the image.
        if self.debug_flag == True:
            image = aruco.drawDetectedMarkers(image, self.marker_corners)

        # Perspective transformation
        if self.marker_corners != []:
            self.four_point_transform(image, self.marker_corners[0][0])
        # Return The current transformed image, or the previous transformed image

        # Calculate the Warped image.
        warped = cv2.warpPerspective(image, self.M, (self.pixel_width, self.pixel_length))


        return warped





def main():

    ap = argparse.ArgumentParser()
    group = ap.add_mutually_exclusive_group()
    group.add_argument("-c", "--camera", type=int, default=0, help="Index of the camera to use")
    group.add_argument("-i", "--image",  type=str, help="Static image to run the algorithm")
    ap.add_argument("-d", "--debug",  action="store_true", default=False, help="Print debbuging images")
    args = vars(ap.parse_args())


    # Start Media Input
    if args["image"] == None:
        # Initialize camera input
        cap = cv2.VideoCapture(args["camera"])
    else:
        # Load image from Disk
        frame_0 = cv2.imread(args["image"])
        if (frame_0 is None):
            print("Error: Image '{}' not Found".format(args["image"]))
            exit()

    # Create the marker tracker object.
    ar_tracker = ARTracker(debug_flag = args["debug"])

    while(True):

        if args["image"] == None:
            # Capture frame-by-frame
            ret, frame = cap.read()
        else:
            frame = frame_0.copy()

        # Take the current frame and warp it.
        warped = ar_tracker.run(frame)

        # Show the images on screen
        cv2.imshow('warped',warped)
        cv2.imshow('frame',frame)
        # Display the resulting frame
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    if args["image"] == None: cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
