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
        self.scan_width = 280
        self.area_width, area_height = 20, 10
        self.roi_vertical_pos = 300
        self.row_begin = (self.scan_height - area_height) // 2
        self.row_end = self.row_begin + area_height
        self.pixel_cnt_threshold = 0.24 * self.area_width * area_height
        self.detect_node = rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        
        self.frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        lbound = np.array([0, 0, self.value_threshold], dtype=np.uint8)
        ubound = np.array([131, 255, 255], dtype=np.uint8)

        # ROI

        frame_L = self.frame[self.roi_vertical_pos:self.roi_vertical_pos + self.scan_height, 0:self.scan_width]
        frame_R = self.frame[self.roi_vertical_pos:self.roi_vertical_pos + self.scan_height, self.image_width - self.scan_width:self.image_width]
        # blur 처리
        blur_L = cv2.GaussianBlur(frame_L, (5, 5), 0)
        blur_R = cv2.GaussianBlur(frame_R, (5, 5), 0)

        # canny 윤곽선
        dst_L = cv2.Canny(blur_L, 50, 200, None, 3)
        dst_R = cv2.Canny(blur_R, 50, 200, None, 3)

        # gray로 나온 canny를 bgr로 변환
        self.cdst_L = cv2.cvtColor(dst_L, cv2.COLOR_GRAY2BGR)
        self.cdst_R = cv2.cvtColor(dst_R, cv2.COLOR_GRAY2BGR)

        # Using Probabilistic Hough Transform
        self.L_lines = cv2.HoughLinesP(dst_L, 1, np.pi / 180, 50, maxLineGap=50)
        self.R_lines = cv2.HoughLinesP(dst_R, 1, np.pi / 180, 50, maxLineGap=50)

        if self.L_lines is not None:
            for i in range(0, len(self.L_lines)):
                l = self.L_lines[i][0]
                cv2.line(self.cdst_L, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 5, cv2.LINE_AA)
        if self.R_lines is not None:
            for j in range(0, len(self.R_lines)):
                r = self.R_lines[j][0]
                cv2.line(self.cdst_R, (r[0], r[1]), (r[2], r[3]), (0, 0, 255), 3, cv2.LINE_AA)

        hsv_L = cv2.cvtColor(self.cdst_L, cv2.COLOR_BGR2HSV)
        self.bin_L = cv2.inRange(hsv_L, lbound, ubound)
        self.view_L = cv2.cvtColor(self.bin_L, cv2.COLOR_GRAY2BGR)

        hsv_R = cv2.cvtColor(self.cdst_R, cv2.COLOR_BGR2HSV)
        self.bin_R = cv2.inRange(hsv_R, lbound, ubound)
        self.view_R = cv2.cvtColor(self.bin_R, cv2.COLOR_GRAY2BGR)

    def detect_lines(self):
        # Return positions of left and right lines detected.
        left, right = -1, -1

        for l in range(self.scan_width, 0, -1):
            area = self.bin_L[self.row_begin:self.row_end, l - self.area_width:l]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                left = self.scan_width - l
                break

        for r in range(0, self.scan_width):
            area = self.bin_R[self.row_begin:self.row_end, r:r + self.area_width]
            if cv2.countNonZero(area) > self.pixel_cnt_threshold:
                right = r
                break

        return left, right



    def show_images(self):
        # Original
        cv2.imshow("frame", self.frame)

        # Detect
        cv2.imshow("frameL", self.view_L)
        cv2.imshow("frameR", self.view_R)

if __name__ == "__main__":
    det = LineDetector()
    time.sleep(1)
    while not rospy.is_shutdown():
        det.show_images(det.detect_lines()[0], det.detect_lines()[1])
