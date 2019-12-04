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
        #if self.obstacle_stop(obs_l, obs_m, obs_r) == 0:
        #    self.driver.drive(90, 90)
        #else:
        #    self.driver.drive(angle + 90, speed + 90)

    def steer(self, left, right):
    if left == -1:
        if 320 < right <= 400:
        angle = -50
        elif 400 < right <= 480:
        angle = -40
        elif 480 < right <= 560:
        angle = -30
        else:
        angle = -20
    # elif 320 < right < 480:
        #        angle = -30

    elif right == -1:
        if 240 < left <= 320:
        angle = 50
        elif 160 < left <= 240:
        angle = 40
        elif 80 < left <= 160:
        angle = 30
        else:
        angle = 20
        
    #elif 160 <= left < 320:
    #    angle = 30

    else:
        angle= 0

    return angle


    def accelerate(self, angle, left, mid, right):
    distance = 50
    
    if min(left, right, mid) < distance:
        speed = 0

        if angle < -40 or angle > 40:
            speed = 30
    elif angle < -30 or angle > 30:
        speed = 25
    elif angle < -20 or angle > 20:
        speed = 25
        else:
            speed = 30
        return speed



    #def printdst(self):
    #    print(obs_l, obs_m, obs_r)

    def exit(self):
        print('finished')

if __name__ == '__main__':
    car = AutoDrive()
    time.sleep(3)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        car.trace()
    #car.printdst()
    rate.sleep()
    rospy.on_shutdown(car.exit)
    
