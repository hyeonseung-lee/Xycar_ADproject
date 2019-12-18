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

        #=======
        self.scan_width2, scan_height2 = 200, 100 #검출 구역
        self.area_width2, area_height2 = 60, 50

        self.roi_vertical_pos2 = 300
        self.row_begin2 = (self.scan_height2 - self.area_height2) // 2
        self.row_end2 = self.row_begin2 + area_height2

        self.pixel_cnt_threshold2 = 0.8 * self.area_width2 * area_height2
        #=========

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

        #==========
        cv_color_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        color_frame = cv_color_image[self.roi_vertical_pos2:self.roi_vertical_pos2 + self.scan_height2, :]

        color_dst = cv2.Canny(color_frame, 50, 100)
        color_cdst = cv2.cvtColor(color_dst, cv2.COLOR_GRAY2BGR)
        color_hsv = cv2.cvtColor(color_cdst, cv2.COLOR_BGR2HSV)

        #red
        red_lbound = np.array([0, 240, 200], dtype=np.uint8)
        red_ubound = np.array([180, 255, 255], dtype=np.uint8)

        self.red_bin = cv2.inRange(color_hsv, red_lbound, red_ubound)
        self.red_view = cv2.cvtColor(self.red_bin, cv2.COLOR_GRAY2BGR)

        #green
        green_lbound = np.array([0, 240, 200], dtype=np.uint8)
        green_ubound = np.array([180, 255, 255], dtype=np.uint8)

        self.green_bin = cv2.inRange(color_hsv, green_lbound, green_ubound)
        self.green_view = cv2.cvtColor(self.green_bin, cv2.COLOR_GRAY2BGR)

        #blue
        blue_lbound = np.array([0, 240, 200], dtype=np.uint8)
        blue_ubound = np.array([180, 255, 255], dtype=np.uint8)

        self.blue_bin = cv2.inRange(color_hsv, blue_lbound, blue_ubound)
        self.blue_view = cv2.cvtColor(self.blue_bin, cv2.COLOR_GRAY2BGR)
        #==========

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

        red, gree, blue= -1, -1, -1

        for i in range(0, self.scan_width):
            red_area = self.red_bin[self.row_begin2:self.row_end2, i:i + self.area_width2]
            green_area = self.green_bin[self.row_begin2:self.row_end2, i:i + self.area_width2]
            blue_area = self.blue_bin[self.row_begin2:self.row_end2, i:i + self.area_width2]

            if cv2.countNonZero(red_area) > self.pixel_cnt_threshold2:
                red= i
                #print("Red")
                break
            if cv2.countNonZero(green_area) > self.pixel_cnt_threshold2:
                #print("Green")
                green= i
                break
            if cv2.countNonZero(blue_area) > self.pixel_cnt_threshold2:
                #print("Blue")
                blue= i
                break
    
        
    

        return left, right, red, green, blue

    

    def show_images(self, left, right, red, green, blue):

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

        #=========
        if red != -1:
            self.redSquare = cv2.rectangle(self.red_view, (red, self.row_begin2),
                (red - self.area_width2, self.row_end2), (255, 255, 0), 3)
            print("red")

        if green != -1:
            self.greenSquare = cv2.rectangle(self.green_view, (green, self.row_begin2),
                (green - self.area_width2, self.row_end2), (255, 255, 0), 3)
            print("green")

        if blue != -1:
            self.blueSquare = cv2.rectangle(self.blue_view, (blue, self.row_begin2),
                (blue - self.area_width2, self.row_end2), (255, 255, 0), 3)
            print("blue")

        cv2.imshow('red_view', self.red_view)
        cv2.imshow('green_view', self.green_view)
        cv2.imshow('blue_view', self.blue_view)
        #===========

        cv2.imshow('view', self.view)
        print(right, left)
        cv2.waitKey(1)

if __name__ == "__main__":
    det = LineDetector()
    time.sleep(1)
    while not rospy.is_shutdown():
        det.show_images(det.detect_lines()[0], det.detect_lines()[1])