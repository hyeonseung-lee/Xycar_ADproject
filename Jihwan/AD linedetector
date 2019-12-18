import rospy, time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class LineDetector:

    def __init__(self, topic):

        self.bridge = CvBridge()
        self.cv_image = np.empty(shape=[0])
        self.value_threshold = 180
        self.image_width = 640
        self.image_middle = 320
        self.scan_height = 40
        self.area_width, area_height = 20, 10
        self.roi_vertical_pos = 300
        self.row_begin = (self.scan_height - area_height) // 2
        self.row_end = self.row_begin + area_height
        self.pixel_cnt_threshold = 0.3 * self.area_width * area_height

        # =======

        self.area_width2, self.area_height2 = 100, 100
        self.roi_vertical_pos2 = 200
        self.pixel_cnt_threshold2 = 0.8 * self.area_width2 * self.area_height2
        # =========

        self.detect_node = rospy.Subscriber(topic, Image, self.conv_image)
        self.last_l, self.last_r = 0, 0

    def conv_image(self, data):

        cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        frame = cv_image[self.roi_vertical_pos:self.roi_vertical_pos + self.scan_height, :]
        dst = cv2.Canny(frame, 50, 100)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

        lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 5)

        if lines is not None:
            for i in range(0, len(lines)):
                l = lines[i][0]
                cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 5, cv2.LINE_AA)

        lbound = np.array([0, 0, self.value_threshold], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)

        hsv = cv2.cvtColor(cdst, cv2.COLOR_BGR2HSV)
        self.bin = cv2.inRange(hsv, lbound, ubound)
        self.view = cv2.cvtColor(self.bin, cv2.COLOR_GRAY2BGR)

        # ==========
        red_lbound = np.array([0, 200, 180], dtype=np.uint8)
        red_ubound = np.array([10, 255, 255], dtype=np.uint8)

        yellow_lbound = np.array([10, 0, 180], dtype=np.uint8)
        yellow_ubound = np.array([30, 255, 255], dtype=np.uint8)

        green_lbound = np.array([50, 0, 180], dtype=np.uint8)
        green_ubound = np.array([70, 255, 255], dtype=np.uint8)


        red_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        red_bin = cv2.inRange(red_hsv, red_lbound, red_ubound)
        self.red_view = cv2.cvtColor(red_bin, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(self.red_view, (0, 200), (100, 300), (255, 0, 255), 3)

        yellow_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        yellow_bin = cv2.inRange(yellow_hsv, yellow_lbound, yellow_ubound)
        self.yellow_view = cv2.cvtColor(yellow_bin, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(self.yellow_view, (0, 200), (100, 300), (255, 0, 255), 3)

        green_hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
        green_bin = cv2.inRange(green_hsv, green_lbound, green_ubound)
        self.green_view = cv2.cvtColor(green_bin, cv2.COLOR_GRAY2BGR)
        cv2.rectangle(self.green_view, (0, 200), (100, 300), (255, 0, 255), 3)

        # ==========
        self.test_view = cv_image.copy()
        cv2.rectangle(self.test_view, (0, 200), (100, 300), (255, 0, 255), 3)
        # ==========

    def detect_lines(self):
        left, right = -1, -1

        for l in range(self.image_middle, self.area_width, -1):
            area = self.bin[self.row_begin:self.row_end, l - self.area_width:l]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                left = l
                break

        for r in range(self.image_middle, self.image_width - self.area_width):
            area = self.bin[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                right = r
                break
        if right > 0 and left > 0 and abs(right - left) < 450:

            if self.last_r == -1:
                right = -1
            elif self.last_l == -1:
                left = -1

        self.last_l = left
        self.last_r = right

        red, yellow, green = False, False, False

        red_area = self.red_view[200:300, 0:100]
        yellow_area = self.yellow_view[200:300, 0:100]
        green_area = self.green_view[200:300, 0:100]

        if cv2.countNonZero(red_area) > self.pixel_cnt_threshold2:
            red = True

        elif cv2.countNonZero(yellow_area) > self.pixel_cnt_threshold2:
            yellow = True

        elif cv2.countNonZero(green_area) > self.pixel_cnt_threshold2:
            green = True

        print(red, yellow, green)

        return left, right, red, yellow, green

    def show_images(self, left, right):

        if left != -1:
            self.lsquare = cv2.rectangle(self.view,
                                         (left, self.row_begin),
                                         (left - self.area_width, self.row_end),
                                         (255, 255, 0), 3)
        else:
            print("Lost left line")

        if right != -1:
            self.rsquare = cv2.rectangle(self.view,
                                         (right, self.row_begin),
                                         (right + self.area_width, self.row_end),
                                         (0, 255, 255), 3)
        else:
            print("Lost right line")


        # ==========
        cv2.imshow('red_view', self.red_view)
        cv2.imshow('yellow_view', self.yellow_view)
        cv2.imshow('green_view', self.green_view)
        # ===========

        cv2.imshow('view', self.view)
        cv2.imshow('origin', self.test_view)

        cv2.waitKey(1)
