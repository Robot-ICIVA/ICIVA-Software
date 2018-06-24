import numpy as np
import cv2
import cv2.aruco as aruco
from collections import OrderedDict
import time
import argparse
import yaml

from http.server import BaseHTTPRequestHandler,HTTPServer
import json
import threading
import os
import sys

from ColorTracker import *
from ARTracker import *


#Create custom HTTPRequestHandler class
class OpenCVHTTPRequestHandler(BaseHTTPRequestHandler):

    #handle GET command
    def do_GET(self):

        global tracked_balls_meter
        try:
            #send code 200 response
            self.send_response(200)

            #send header first
            self.send_header('Content-type','text-html')
            self.end_headers()

            #send file content to client
            t = json.dumps(tracked_balls_meter)
            # t = json.dumps([[1,2,3],[4,5,6],[7,8,9]])
            self.wfile.write(bytes(t,"utf-8"))
            return

        except IOError:
            self.send_error(404, 'file not found')

def run():

    global server_on

    print('http server is starting...')

    #ip and port of servr
    #by default http server port is 8000
    server_address = ('127.0.0.1', 8000)
    httpd = HTTPServer(server_address, OpenCVHTTPRequestHandler)
    print('http server is running...')
    while(server_on):
        httpd.handle_request()
        # httpd.serve_forever()

# CV2 Callback
def onMouse (event, x, y, f, other):
    # Bring the image from the global context
    global col_tracker

    # print("x:{}  y:{},  color = {}".format(x,y, image[y][x]))
    # Set thy callback event
    if (event == cv2.EVENT_LBUTTONDOWN):
        col_tracker.tracked_colors.append( col_tracker.shifted[y][x] )


def track_ar_marker(x):
    """Start (1) or  Stop (0) the Perspective recalculation"""
    global ar_tracker

    if x == 1:
        ar_tracker.Compute_M = True
    else:
        ar_tracker.Compute_M = False

def field_length(x):
    """Defines the length of the extended perspective corrected field"""
    global ar_tracker

    ar_tracker.field_length = x/100.0

def field_width(x):
    """Defines the length of the extended perspective corrected field"""
    global ar_tracker

    ar_tracker.field_width = x/100.0

def reset_tracked_colors(x):
    """Resets the list of tracked colors to None"""
    global col_tracker

    col_tracker.tracked_colors = []





def main():

    # construct the argument parse and parse the arguments
    ap = argparse.ArgumentParser()
    group = ap.add_mutually_exclusive_group()
    group.add_argument("-c", "--camera", type=int, default=0, help="Index of the camera to use")
    group.add_argument("-i", "--image",  type=str, help="Static image to run the algorithm")
    ap.add_argument("-d", "--debug",  action="store_true", default=False, help="Print debbuging images")
    group.add_argument("-m", "--matrix",  type=str, default="calibration.yaml",help="Calibration file")
    args = vars(ap.parse_args())

    # Load calibration parameters
    with open(args["matrix"]) as f:
        loadeddict = yaml.load(f)
    # Unpack calibration parameters
    mtx  = np.asarray(loadeddict.get('camera_matrix'))
    dist = np.asarray(loadeddict.get('dist_coeff'))


    # Define  global variables
    global col_tracker, ar_tracker, tracked_balls_meter, server_on
    # Initialize the Tracked balls empty
    tracked_balls_meter = []
    # Initialize the "KILL THE SERVER" flag, so that the server can run.
    server_on = True
    # Create NamedWindow, and set callback.
    cv2.namedWindow('Corrected Perspective', cv2.WINDOW_NORMAL)

    cv2.setMouseCallback('Corrected Perspective', onMouse, 0 );

    # Initialize the GUI.
    cv2.createTrackbar('Track AR marker', 'Corrected Perspective', 1, 1, track_ar_marker)
    cv2.createTrackbar('Field Length [cm]', 'Corrected Perspective', 78, 150, field_length)
    cv2.createTrackbar('Field Width [cm]', 'Corrected Perspective', 78, 150, field_width)
    cv2.createTrackbar('Reset Tracked Colors', 'Corrected Perspective', 0, 1, reset_tracked_colors)

    # Start Media Input
    if args["image"] == None:
        # Initialize camera input
        cap = cv2.VideoCapture(args["camera"])
        ret, calib_frame = cap.read()
    else:
        # Load image from Disk
        frame_0 = cv2.imread(args["image"])
        calib_frame = frame_0.copy()
        if (frame_0 is None):
            print("Error: Image '{}' not Found".format(args["image"]))
            exit()

    # Get the calibration matrix
    h,  w = calib_frame.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))


    # Create the marker tracker object.
    # Initialize color tracking object
    ar_tracker = ARTracker(debug_flag = args["debug"], marker_size = 0.151) # 15.1 cm
    col_tracker = ColorTracker(debug_flag = args["debug"])

    t = threading.Thread(target=run)
    t.start()

    while (1):

        if args["image"] == None:
            # Capture frame-by-frame
            ret, frame = cap.read()
        else:
            frame = frame_0.copy()

        # Start the FPS timer
        start_time = time.time()

        # Undistort and Crop the image
        dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)
        x,y,w,h = roi
        frame = dst[y:y+h, x:x+w]

        # Take the current frame and warp it.
        warped = ar_tracker.run(frame)
        # Then track the balls.
        tracked_balls = col_tracker.run(warped)

        # Draw the amount of tracked balls.
        cv2.putText(warped, "Tracked Balls = {}".format(len(tracked_balls)), (2, 22), cv2.FONT_HERSHEY_SIMPLEX,0.8, (255, 255, 255), 2)

        ## Draw the tracked contours on screen, and convert the data from pixels to meters
        intermediary_ball_list = []

        for ball in tracked_balls:
            # Draw a circle around each ball
            center = ball[0]
            radius = ball[1]
            color = col_tracker.RGB_dictionary[ball[2]]
            cv2.circle(warped,center,radius,(color[2], color[1], color[0]),2)

            # Draw a white dot at the center of each ball
            cv2.circle(warped, center, 1, (255, 255, 255), -1)

            center_m = (center[0] * ar_tracker.pixel_to_meters, (warped.shape[0] - center[1])  * ar_tracker.pixel_to_meters)
            radius = radius * ar_tracker.pixel_to_meters
            intermediary_ball_list.append([center_m, radius, ball[2]])

        # Finally Write all this information to a "global" variable
        tracked_balls_meter = intermediary_ball_list.copy()
        if args["debug"]: print("tracked_balls = {}".format(tracked_balls_meter))

        end_time = time.time()
        # Calculate speed of the algorithm
        cv2.putText(warped, "FPS = {}".format(round(1/(end_time-start_time),1)), (2, warped.shape[0]-2), cv2.FONT_HERSHEY_SIMPLEX,0.5, (255, 255, 255), 2)

        # Redraw the Image
        cv2.imshow("Original", frame)
        # # Shrink the image to fit it in an acceptable screen space.
        # warped = cv2.resize(warped,None,fx=.5, fy=.5, interpolation = cv2.INTER_AREA)
        cv2.imshow("Corrected Perspective",  warped)



        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # When everything done, release the capture
    # When everything done, release the capture
    if args["image"] == None: cap.release()
    cv2.destroyAllWindows()
    server_on = False
    print("Terminating Server ...")


if __name__ == '__main__':
    main()
