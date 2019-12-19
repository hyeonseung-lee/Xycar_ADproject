import rospy
import cv2, time
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class LineDetector:

	def __init__(self, topic):
		self.cam_img = np.zeros(shape=(480, 640, 3), dtype=np.uint8)
		self.mask = np.zeros(shape=(self.scan_height, self.image_width), dtype=np.uint8)
		self.edge = np.zeros(shape=(self.scan_height, self.image_width), dtype=np.uint8)
		self.bridge = CvBridge()
		rospy.Subscriber(topic, Image, self.conv_image)

		image_middle = 320
		area_width, area_height = 20, 10
		roi_vertical_pos = 300
		row_begin = (self.scan_height - self.area_height) // 2
		row_end = self.row_begin + self.area_height
		pixel_cnt_threshold = 0.2 * self.area_width * self.area_height


	def conv_image(self, data):
		self.cam_img= self.bridge.imgmsg_to_cv2(data, 'bgr8')
		v = self.roi_vertical_pos
		roi = self.cam_img[v:v + self.scan_height, :]

		blur = cv2.GaussianBlur(roi, (5, 5), 0)
		dst = cv2.Canny(blur, 50, 200, None, 3)
		cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)

		lines = cv2.HoughLinesP(dst, 1, np.pi / 180, 50, None, 50, 10)

		if lines is not None:
			for i in range(0, len(lines)):
				l = lines[i][0]
				cv2.line(cdst, (l[0], l[1]), (l[2], l[3]), (0, 0, 255), 3, cv2.LINE_AA)

		hsv = cv2.cvtColor(cdst, cv2.COLOR_BGR2HSV)

		avg_value = np.average(hsv[:, :, 2])
		value_threshold = avg_value * 1.0

		lbound = np.array([0, 0, value_threshold], dtype=np.uint8)
		ubound = np.array([131, 255, 255], dtype=np.uint8)

		bin = cv2.inRange(hsv, lbound, ubound)

		view= cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)


	
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

		if left != -1:
			lsquare = cv2.rectangle(self.view, (left, self.row_begin),
                                (left - self.area_width, self.row_end), (0, 255, 0), 3)
		else:
			print("Lost left line")


		if right != -1:
			rsquare = cv2.rectangle(self.view, (right, self.row_begin),
                                (right + self.area_width, self.row_end), (0, 255, 0), 3)
		else:
			print("Lost right line")

		return left, right



	def show_images(self, left, right):
		cv2.imshow("Source", self.roi)

		cv2.imshow("bin", self.bin)



