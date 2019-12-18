import rospy, time
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

#그린, 블루 바운드값 hsv조절해야함
#ROI값 조절해야함
#autodirve 코드랑 launch에 인자값 추가해야함

class ColorDetector:
	def __init__(self, topic):
		self.bridge = CvBridge()
        self.cv_image = np.empty(shape=[0])

        self.image_width = 640

        self.scan_width, scan_height = 200, 100 #검출 구역
        self.area_width, area_height = 60, 50

        self.roi_vertical_pos = 300
        self.row_begin = (self.scan_height - self.area_height) // 2
        self.row_end = self.row_begin + area_height

        self.pixel_cnt_threshold = 0.8 * self.area_width * area_height
        self.detect_node = rospy.Subscriber(topic, Image, self.conv_color_image)

    def conv_color_image(self, data):
    	cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
    	frame = cv_image[self.roi_vertical_pos:self.roi_vertical_pos + self.scan_height, :]

		dst = cv2.Canny(frame, 50, 100)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)

        #red
        red_lbound = np.array([0, 240, 200], dtype=np.uint8)
		red_ubound = np.array([180, 255, 255], dtype=np.uint8)

		self.red_bin = cv2.inRange(hsv, red_lbound, red_ubound)
		self.red_view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

		#green
		green_lbound = np.array([0, 240, 200], dtype=np.uint8)
		green_ubound = np.array([180, 255, 255], dtype=np.uint8)

		self.green_bin = cv2.inRange(hsv, green_lbound, green_ubound)
		self.green_view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)

		#blue
		blue_lbound = np.array([0, 240, 200], dtype=np.uint8)
		blue_ubound = np.array([180, 255, 255], dtype=np.uint8)

		self.blue_bin = cv2.inRange(hsv, blue_lbound, blue_ubound)
		self.blue_view = cv2.cvtColor(bin, cv2.COLOR_GRAY2BGR)


	def detect_color(self):
		red, gree, blue= -1, -1, -1

		for i in range(0, self.scan_width):
			red_area = self.red_bin[self.row_begin:self.row_end, i:i + self.area_width]
			green_area = self.green_bin[self.row_begin:self.green_end, i:i + self.area_width]
			blue_area = self.blue_bin[self.row_begin:self.blue_end, i:i + self.area_width]

			if cv2.countNonZero(red_area) > self.pixel_cnt_threshold:
				red= i
				print("Red")
				break
			if cv2.countNonZero(green_area) > self.pixel_cnt_threshold:
				print("Green")
				green= i
				break
			if cv2.countNonZero(blue_area) > self.pixel_cnt_threshold:
				print("Blue")
				blue= i
				break

		return red, green, blue

	def show_color_images(self, red, green, blue):
		if red != -1:
			self.redSquare = cv2.rectangle(self.red_view, (red, self.row_begin),
				(red - self.area_width, self.row_end), (255, 255, 0), 3)

		if green != -1:
			self.greenSquare = cv2.rectangle(self.green_view, (green, self.row_begin),
				(green - self.area_width, self.row_end), (255, 255, 0), 3)

		if blue != -1:
			self.blueSquare = cv2.rectangle(self.blue_view, (blue, self.row_begin),
				(blue - self.area_width, self.row_end), (255, 255, 0), 3)

		cv2.imshow('red_view', self.red_view)
		cv2.imshow('green_view', self.green_view)
		cv2.imshow('blue_view', self.blue_view)

		cv2.waitKey(1)
