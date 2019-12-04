#!/usr/bin/env python

import rospy, time

from linedetector import LineDetector
from obstacledetector import ObstacleDetector
from motordriver import MotorDriver

class AutoDrive:

    def __init__(self):
        rospy.init_node('xycar_driver')
        self.line_detector = LineDetector('/usb_cam/image_raw')
        self.driver = MotorDriver('/xycar_motor_msg')
    self.obstacle_detector = ObstacleDetector('/ultrasonic')
    
    def trace(self):
    obs_l, obs_m, obs_r = self.obstacle_detector.get_distance()
        line_l, line_r = self.line_detector.detect_lines()
        self.line_detector.show_images(line_l, line_r)
        angle = self.steer(line_l, line_r)
        speed = self.accelerate(angle, obs_l, obs_m, obs_r)
    self.driver.drive(angle + 90, speed + 90)
    
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
    print(mid)
    
    if mid < 40:
        speed = 0
        return speed

    if angle <= -40 or angle >= 40:
        speed = 30
    elif angle <= -25 or angle >= 25:
        speed = 40
    else:
        speed = 50

    return speed
    
    def exit(self):
        print('finished')
        
if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
    rate.sleep()
    rospy.on_shutdown(car.exit)
    
