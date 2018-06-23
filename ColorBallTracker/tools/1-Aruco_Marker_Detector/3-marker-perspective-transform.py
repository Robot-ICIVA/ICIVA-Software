import numpy as np
import cv2
import cv2.aruco as aruco
import argparse


def four_point_transform(image, pts, side_meters):
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

    # now that we have the dimensions of the new image, construct
    # the set of destination points to obtain a "birds eye view",
    # (i.e. top-down view) of the image, again specifying points
    # in the top-left, top-right, bottom-right, and bottom-left
    # order
    dst = np.array([
        [0, side - 1],
        [0,0],
        [side - 1, 0],
        [side - 1, side - 1]], dtype = "float32")


    # compute the perspective transform matrix and then apply it
    M = cv2.getPerspectiveTransform(pts, dst)
    warped = cv2.warpPerspective(image, M, (side, side))

    # compute pixel to distance ratio.
    pixel_to_meters =  side_meters / side

    # return the warped image
    return warped, M, pixel_to_meters




ap = argparse.ArgumentParser()
ap.add_argument("-c", "--camera", required=True,
	help="path to the input camera")
args = vars(ap.parse_args())

# Initialize camera input
cap = cv2.VideoCapture(int(args["camera"]))


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # print(frame.shape) #480x640
    # Our operations on the frame come here
    aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
    parameters =  aruco.DetectorParameters_create()

    # Detect Markers and print the corners of the found ones
    corners, ids, rejectedImgPoints = aruco.detectMarkers(frame, aruco_dict, parameters=parameters)
    print(corners)

    # Draw the found markers in the image, and show the image.
    frame = aruco.drawDetectedMarkers(frame, corners)

    # Perspective transformation
    if corners != []:
        warped, M, _ = four_point_transform(frame, corners[0][0], 0.144)
        cv2.imshow('warped',warped)


    # Display the resulting frame
    cv2.imshow('frame',frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
