#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from motordriver import MotorDriver
from imuread import ImuRead


class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')
        self.imu = ImuRead('/diagnostics')
        self.information = {"speed": -1, "signal": "", "=>": "", "pitch": -1, "roll": -1, "yaw": -1}

    def trace(self):
        line_l, line_r, red, yellow, green = self.line_detector.detect_lines()
        self.r, self.y, self.g = red, yellow, green
        self.line_detector.show_images(line_l, line_r)
        self.information["roll"], self.information["pitch"], self.information["yaw"] = self.imu.get_data()
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, red, yellow, green)
        self.information["speed"] = speed + 90
        self.driver.drive(angle + 90, speed + 90)

        print(speed + 90)
        print(r, p, y)

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

    def accelerate(self, angle, red, yellow, green):

        # if red is detected, stop
        if red and (not yellow) and (not green):
            speed = 0
        # if yellow is detected, go low speed
        elif (not red) and yellow and (not green):
            speed = 20
        # if green is detected, go origin speed
        elif (not red) and (not yellow) and green:
            speed = 50

        # This is origin code
        else:  # (not red) and (not yellow) and (not green):
            if angle <= -40 or angle >= 40:
                speed = 30
            elif angle <= -25 or angle >= 25:
                speed = 40
            else:
                speed = 50

        print(red, yellow, green)

        return speed

    def printInformation(self):

        if self.r:
            self.information["signal"] = "red"
            self.information["=>"] = "stop!"
        if self.y:
            self.information["signal"] = "yellow"
            self.information["=>"] = "slow"
        if self.g:
            self.information["signal"] = "green"
            self.information["=>"] = "just go"

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
