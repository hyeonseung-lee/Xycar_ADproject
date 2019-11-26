#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture("2.avi")

value_threshold = 180

image_width = 640
image_middle = 320
scan_width, scan_height = 200, 40
lmid, rmid = scan_width, image_width - scan_width
area_width, area_height = 20, 10
roi_vertical_pos = 300
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_cnt_threshold = 0.2 * area_width * area_height



while True:
    ret, frame = cap.read()
    # ROI 설정
    frame = frame[roi_vertical_pos:roi_vertical_pos + scan_height, :]
	
	# blur 처리    
	blur = cv2.GaussianBlur(frame, (5, 5), 0)
	
	# canny 윤곽선    
	dst = cv2.Canny(blur, 50, 200, None, 3)
	
	# gray로 나온 canny를 bgr로 변환    
	cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

    # Using Probabilistic Hough Transform
    lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

    if lines is not None:
        for i in range(0, len(lines)):
            l = lines[i][0]
            cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)
	
    # Setting range to binary
    lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    # Convert BGR to HSV
    hsv = cv2.cvtColor(cdst, cv2.COLOR_BGR2HSV)

    # Binary
    bin = cv2.inRange(hsv, lbound, ubound)

    # Gray
    gray = cv2.cvtColor(cdst, cv2.COLOR_BGR2GRAY)


    #################################################################################

    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

    left, right = -1, -1

    # Start at middle
    for l in range(image_middle, area_width, -1):
        area = bin[row_begin:row_end, l - area_width:l]
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            left = l
            break

    for r in range(image_middle, image_width - area_width):
        area = bin[row_begin:row_end, r:r + area_width]
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            right = r
            break

    if left != -1:
        lsquare = cv2.rectangle(view,
                                (left, row_begin),
                                (left - area_width, row_end),
                                (0, 255, 0), 3)
    else:
        print("Lost left line")

    if right != -1:
        rsquare = cv2.rectangle(view,
                                (right, row_begin),
                                (right + area_width, row_end),
                                (0, 255, 0), 3)
    else:
        print("Lost right line")


    # Original
    cv2.imshow("Source", frame)

    # Proccessed by PHT
    #cv2.imshow("PHT", cdst)

    #cv2.imshow("gray", gray)

    #cv2.imshow("hsv", hsv)

    cv2.imshow("bin", bin)

    cv2.imshow("square", view)

    # Press q to exit
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.03)

cap.release()
cv2.destroyAllWindows()
