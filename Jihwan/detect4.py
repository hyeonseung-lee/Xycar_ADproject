#!/usr/bin/env python

import rospy, time
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge
import numpy as np

#cap = cv2.VideoCapture('/home/nvidia/xycar/src/auto_drive/src/2.avi')

bridge = CvBridge()
#rate = rospy.Rate(15)
cv_image = np.empty(shape=[0])

value_threshold = 180

image_width = 640
image_middle = 320
scan_height = 40
area_width, area_height = 20, 10
roi_vertical_pos = 300
row_begin = (scan_height - area_height) // 2
row_end = row_begin + area_height
pixel_cnt_threshold = 0.2 * area_width * area_height


def callback(img_data):
    global bridge
    global cv_image
    cv_image = bridge.imgmsg_to_cv2(img_data, "bgr8")


#ret, frame = cap.read()

if __name__ == "__main__":
    rospy.init_node("camtest_node")
    rospy.Subscriber("/usb_cam/image_raw", Image, callback)
    time.sleep(1)
    while not rospy.is_shutdown():
        cv2.imshow("camera", cv_image)
        frame = cv_image[roi_vertical_pos:roi_vertical_pos + scan_height, :]
        blur = cv2.GaussianBlur(frame, (5, 5), 0)
        dst = cv2.Canny(blur, 50, 200, None, 3)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)

        hsv = cv2.cvtColor(cdst, cv2.COLOR_BGR2HSV)
        bin = cv2.inRange(hsv, lbound, ubound)
        view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

        left, right = -1, -1

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

        cv2.imshow('bin', bin)
        cv2.imshow('view', view)

        if cv2.waitKey(1) & 0xff == ord("q"):
            break

    cv2.destroyAllWindows()
