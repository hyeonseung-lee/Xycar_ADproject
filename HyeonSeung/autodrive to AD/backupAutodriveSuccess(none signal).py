#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver
from imuread import ImuRead


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.obstacle_detector = ObstacleDetector('/ultrasonic')
        self.imu = ImuRead('/diagnostics')
        self.information = {"speed": -1, "signal": "", "=>": "", "pitch": -1, "roll": -1, "yaw": -1}

    def trace(self):
        obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l, line_r = self.line_detector.detect_lines()
        # color_red, color_green, color_blue = self.line_detector.detect_colors()

        self.line_detector.show_images(line_l, line_r)
        self.information["roll"], self.information["pitch"], self.information["yaw"] = self.imu.get_data()
        # self.line_detector.show_colors(color_red, color_green, color_blue)
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
        self.information["speed"] = speed + 90
        # print('R (%.1f), P (%.1f), Y (%.1f)' % (self.r, self.p, self.y))
        self.driver.drive(angle + 90, speed + 90)

    # if self.obstacle_stop(obs_l, obs_m, obs_r) == 0:
    #    self.driver.drive(90, 90)
    # else:
    #    self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
        if left == -1:
            if 320 < right < 390:
                angle = -50
            else:
                angle = (550 - right) / (-3)
        elif right == -1:
            if 250 < left < 320:
                angle = 50
            else:
                angle = (left - 90) / 3
        else:
            angle = 0
        return angle

    def accelerate(self, angle, left, mid, right):
        # print(mid)
        if 0 < mid < 85:
            speed = 0
            return speed

        if angle <= -40 or angle >= 40:
            speed = 30
        elif angle <= -25 or angle >= 25:
            speed = 40
        else:
            speed = 50

        return speed

    def printInformation(self):
        print("speed : " + str(self.information["speed"]) + " ", "signal : " + str(self.information["signal"]) + " ",
              "=> " + str(self.information["=>"]))
        print("pitch : " + str(self.information["pitch"]) + " ", "roll : " + str(self.information["roll"]) + " ",
              "yaw : " + str(self.information["yaw"]))
        print("")

    def exit(self):
        print('finished')


if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
        car.printInformation()
        rate.sleep()
    rospy.on_shutdown(car.exit)
