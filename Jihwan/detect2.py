#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture("2.avi")

while True:
    ret, frame = cap.read()
    frame = frame[300:340, :]
    blur = cv2.GaussianBlur(frame, (5, 5), 0)
    dst = cv2.Canny(blur, 50, 200, None, 3)
    cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

    # Using Probabilistic Hough Transform
    linesP = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

    if linesP is not None:
        for i in range(0, len(linesP)):
            l = linesP[i][0]
            cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

    # Setting range to binary
    lbound = np.array([0, 0, 180], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cdst, cv2.COLOR_BGR2HSV)

    # Binary
    bin = cv2.inRange(hsv, lbound, ubound)

    # Gray
    gray = cv2.cvtColor(cdst, cv2.COLOR_BGR2GRAY)


    # Original
    cv2.imshow("Source", frame)

    # Proccessed by PHT
    cv2.imshow("PHT", cdst)

    cv2.imshow("gray", gray)

    cv2.imshow("hsv", hsv)

    cv2.imshow("bin", bin)

    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.02)

cap.release()
cv2.destroyAllWindows()
