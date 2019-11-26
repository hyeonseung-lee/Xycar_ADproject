#!/usr/bin/env python
import cv2, time
import numpy as np

image_width = 640
image_middle = 320
scan_width, scan_height = 200, 40
lmid, rmid = scan_width, image_width - scan_width
area_width, area_height = 20, 10
roi_vertical_pos = 300
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_cnt_threshold = 0.1 * area_width * area_height

cap = cv2.VideoCapture("2.avi")

while True:
    ret, frame = cap.read()
    frame = frame[roi_vertical_pos:roi_vertical_pos + scan_height, :]
    cny = cv2.Canny(frame, 100, 200)
    cdst = cv2.cvtColor(cny, cv2.COLOR_GRAY2BGR)

    lbound = np.array([0, 0, 200], dtype=np.uint8)
    ubound = np.array([131, 131, 255], dtype=np.uint8)
    hsv = cv2.cvtColor(cdst, cv2.COLOR_BGR2HSV)

    bin = cv2.inRange(hsv, lbound, ubound)
    view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)
    left, right = -1, -1

    for l in range(image_middle - 100, area_width, -1):
        area = bin[row_begin:row_end, l - area_width:l]
        if cv2.countNonZero(area) > pixel_cnt_threshold:
            left = l
            break

    for r in range(image_middle + 100, image_width - area_width):
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

    cv2.imshow("Source", frame)
    cv2.imshow("square", view)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

    time.sleep(0.03)

cap.release()
cv2.destroyAllWindows()