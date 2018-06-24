import numpy as np
import cv2
import cv2.aruco as aruco
import argparse


def four_point_transform(image, pts, oldM, side_meters, field_length, field_width):
    # obtain a consistent order of the points and unpack them
    # individually

    # Calculate the size of each of the sides of the square,
    # and use the maximum for the reference size of the warped
    # image.
    side1 = np.linalg.norm(pts[0]-pts[1])
    side2 = np.linalg.norm(pts[1]-pts[2])
    side3 = np.linalg.norm(pts[2]-pts[3])
    side4 = np.linalg.norm(pts[3]-pts[0])

    side = max(side1, side2, side3, side4)


    # Compute pixel to distance ratio.
    pixel_to_meters = side_meters / side

    # Compute pixel size of the resulting Image.
    pixel_length = int(field_length/pixel_to_meters)
    pixel_width = int(field_width/pixel_to_meters)

    # Calculate the destination points for the transformation.
    dst = np.array([
    [0, pixel_length],
    [0,pixel_length - (side-1)],
    [side - 1, pixel_length - (side-1)],
    [side - 1, pixel_length]], dtype = "float32")

    # Compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(pts, dst)

    # Check that the change between transformation matrices is more than 2% before converting
    # norm_M = np.linalg.norm(M)
    # norm_oldM = np.linalg.norm(oldM)
    # if abs((norm_M - norm_oldM)/norm_M) < 0.1:
    #     M = oldM

    warped = cv2.warpPerspective(image, M, (pixel_width, pixel_length))


    # Return the warped image
    return warped, M, pixel_to_meters




ap = argparse.ArgumentParser()
ap.add_argument("-c", "--camera", required=True,
	help="path to the input camera")
args = vars(ap.parse_args())

# Initialize camera input
cap = cv2.VideoCapture(int(args["camera"]))


# Our operations on the frame come here
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
parameters =  aruco.DetectorParameters_create()
# Activate subpixel corner refinement.
parameters.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX
parameters.cornerRefinementMinAccuracy = 0.1

# Initialize M
M = 0

while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()

    # Detect Markers and print the corners of the found ones
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    # print(corners)

    # Draw the found markers in the image, and show the image.
    frame = aruco.drawDetectedMarkers(frame, corners)


    # Perspective transformation
    if corners != []:
        warped, M, _ = four_point_transform(frame, corners[0][0], M, 0.144, 0.46, 0.46)

        cv2.imshow('warped',warped)


    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
