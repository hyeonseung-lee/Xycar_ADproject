#!/usr/bin/env python

import cv2, time
import numpy as np

cap = cv2.VideoCapture("2.avi")

value_threshold = 180

image_width = 640
image_middle = 320
scan_width, scan_height = 200, 100
lmid, rmid = scan_width, image_width - scan_width
area_width, area_height = 20, 10
roi_vertical_pos = 300
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_cnt_threshold = 0.2 * area_width * area_height



while True:
    ret, frame = cap.read()
    # ROI

    frame_L = frame[roi_vertical_pos:roi_vertical_pos + scan_height, 0:scan_width]
    frame_R = frame[roi_vertical_pos:roi_vertical_pos + scan_height, image_width - scan_width:image_width]
    # blur 처리    
    blur_L = cv2.GaussianBlur(frame_L, (5, 5), 0)
    blur_R = cv2.GaussianBlur(frame_R, (5, 5), 0)
	
    # canny 윤곽선    
    dst_L = cv2.Canny(blur_L, 50, 200, None, 3)
    dst_R = cv2.Canny(blur_R, 50, 200, None, 3)
	
    # gray로 나온 canny를 bgr로 변환    
    cdst_L = cv2.cvtColor(dst_L, cv2.COLOR_GRAY2BGR)
    cdst_R = cv2.cvtColor(dst_R, cv2.COLOR_GRAY2BGR)

    # Using Probabilistic Hough Transform
    L_lines = cv2.HoughLinesP(dst_L, 1, np.pi / 180, 50, maxLineGap=50)
    R_lines = cv2.HoughLinesP(dst_R, 1, np.pi / 180, 50, maxLineGap=50)

    Lx1, Ly1, Lx2, Ly2 = 0, 0, 0, 0
    Rx1, Ry1, Rx2, Ry2 = 0, 0, 0, 0
    if L_lines is not None:
        for line in L_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cdst_L, (x1, y1), (x2, y2), (0, 0, 255), 5)
            Lx1, Ly1, Lx2, Ly2 = x1, y1, x2, y2
            lGradient = (Lx2 - Lx1) / (Ly2 - Ly1)
            print("lGradient", lGradient)       # 음수
    if R_lines is not None:
        for line in R_lines:
            x1, y1, x2, y2 = line[0]
            cv2.line(cdst_R, (x1, y1), (x2, y2), (0, 0, 255), 5)
            Rx1, Ry1, Rx2, Ry2 = x1, y1, x2, y2
            rGradient = (Rx2 - Rx1) / (Ry2 - Ry1)
            print("rGradient", rGradient)       # 양수

    sumGradient = rGradient + lGradient
                    #양수         음수

    #return Lx1, Ly1, Lx2, Ly2, Rx1, Ry1, Rx2, Ry2
    # Setting range to binary
    lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
    ubound = np.array([131, 255, 255], dtype=np.uint8)

    # Convert BGR to HSV
    Lhsv = cv2.cvtColor(cdst_L, cv2.COLOR_BGR2HSV)
    Rhsv = cv2.cvtColor(cdst_R, cv2.COLOR_BGR2HSV)

    # Binary
    Lbin = cv2.inRange(Lhsv, lbound, ubound)
    Rbin = cv2.inRange(Rhsv, lbound, ubound)

    # Gray
    Lgray = cv2.cvtColor(cdst_L, cv2.COLOR_BGR2GRAY)
    Rgray = cv2.cvtColor(cdst_R, cv2.COLOR_BGR2GRAY)


    #################################################################################

    Lview = cv2.cvtColor(Lbin, cv2.COLOR_GRAY2BGR)
    Rview = cv2.cvtColor(Rbin, cv2.COLOR_GRAY2BGR)

    """
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

    """
    # Original
    cv2.imshow("frame", frame)

    # Detect
    cv2.imshow("frameL", cdst_L)
    cv2.imshow("frameR", cdst_R)
    # Press q to exit
    key = cv2.waitKey(1)
    if key == 27:
        break



    time.sleep(0.001)

cap.release()
cv2.destroyAllWindows()
