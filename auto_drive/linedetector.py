import rospy
import cv2, time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineDetector:

    def __init__(self, topic):
        # Initialize various class-defined attributes, and then...
        self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
        self.mask = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.edge = np.zeros(shape=(self.scan_height, self.image_width),
                             dtype=np.uint8)
        self.bridge = CvBridge() # ros image message to openCV
        rospy.Subscriber(topic, Image, self.conv_image)

    def conv_image(self, data):
        self.cam_img = self.bridge.imgmsg_to_cv2(data, 'bgr8')
        v = self.roi_vertical_pos
        roi = self.cam_img[v:v + self.scan_height, :]

        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        avg_value = np.average(hsv[:, :, 2])
        value_threshold = avg_value * 1.0
        lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
        ubound = np.array([100, 255, 255], dtype=np.uint8)
        self.mask = cv2.inRange(hsv, lbound, ubound)

        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        self.edge = cv2.Canny(blur, 60, 70)

    def detect_lines(self):
        # Initialize various class-defined attributes
        self.cap = cv2.VideoCapture(0)
        value_threshold = 180

        self.image_width = 640
        image_middle = 320
        self.scan_width, self.scan_height = 200, 40
        self.area_width, self.area_height = 20, 10
        self.roi_vertical_pos = 300
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + self.area_height
        pixel_cnt_threshold = 0.2 * self.area_width * self.area_height



        # Return positions of left and right lines detected.
        while True:
            ret, frame = self.cap.read()
            self.frame = frame[self.roi_vertical_pos:self.roi_vertical_pos + self.scan_height, :]
            blur = cv2.GaussianBlur(frame, (5, 5), 0)
            dst = cv2.Canny(blur, 50, 200, None, 3)
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

            #################################################################################

            self.view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

            left, right = -1, -1

            # Start at middle
            for l in range(image_middle, self.area_width, -1):
                area = bin[self.row_begin:self.row_end, l - self.area_width:l]
                if cv2.countNonZero(area) > pixel_cnt_threshold:
                    left = l
                    break

            for r in range(image_middle, self.image_width - self.area_width):
                area = bin[self.row_begin:self.row_end, r:r + self.area_width]
                if cv2.countNonZero(area) > pixel_cnt_threshold:
                    right = r
                    break

            return left, right

            if left != -1:
                lsquare = cv2.rectangle(self.mask,
                                        (left, row_begin),
                                        (left - area_width, row_end),
                                        (0, 255, 0), 3)
            else:
                print("Lost left line")

            if right != -1:
                rsquare = cv2.rectangle(self.mask,
                                        (right, row_begin),
                                        (right + area_width, row_end),
                                        (0, 255, 0), 3)
            else:
                print("Lost right line")

                # Original
            cv2.imshow("Source", frame)

            cv2.imshow("bin", bin)

            cv2.imshow("square", view)

            # Press q to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            time.sleep(0.03)

    def show_images(self, left, right):
        # Display images for debugging purposes;
        cv2.imshow("cam_img", self.cam_img)
        cv2.imshow("mask", self.mask)
        self.cap.release()
        cv2.waitKey()
        cv2.destroyAllWindows()
