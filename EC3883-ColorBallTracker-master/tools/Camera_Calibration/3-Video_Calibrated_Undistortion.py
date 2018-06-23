import numpy as np
import cv2
import yaml
import argparse

with open('calibration.yaml') as f:
    loadeddict = yaml.load(f)

# Unpack calibration parameters
mtx  = np.asarray(loadeddict.get('camera_matrix'))
dist = np.asarray(loadeddict.get('dist_coeff'))

# Retrieve arguments from CLI
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


while(True):
    # Capture frame-by-frame
    ret, frame = cap.read()
    # print(frame.shape) #480x640
    original = frame.copy()

    h,  w = frame.shape[:2]
    newcameramtx, roi=cv2.getOptimalNewCameraMatrix(mtx,dist,(w,h),1,(w,h))

    # undistort
    dst = cv2.undistort(frame, mtx, dist, None, newcameramtx)

    # crop the image
    x,y,w,h = roi
    dst = dst[y:y+h, x:x+w]

    print(roi)
    # Display the resulting frame
    cv2.imshow('undistorted',frame)
    cv2.imshow('original',original)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# When everything done, release the capture
cap.release()
cv2.destroyAllWindows()
