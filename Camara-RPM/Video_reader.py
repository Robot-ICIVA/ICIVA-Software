import numpy as np
import cv2

cap = cv2.VideoCapture('Video/3_26/output2.avi')

while(cap.isOpened()):
    ret, frame = cap.read()

    #gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if (type(frame) != type(None)):
        cv2.imshow('frame',frame)
        if cv2.waitKey(1000) & 0xFF == ord('q'):
            break
    else:
        print("Finished")
        break
cap.release()
cv2.destroyAllWindows()