#!/usr/bin/env python

import rospy, time
from std_msgs.msg import Int32MultiArray

motor_pub = None
usonic_data = None

def init_node():
    global motor_pub
    rospy.init_node('sample')
    rospy.Subscriber('ultrasonic', Int32MultiArray, callback)
    motor_pub = rospy.Publisher('xycar_motor_msg', Int32MultiArray, queue_size=1)

def exit_node():
    print('finished')

def drive(angle, speed):
    global motor_pub
    drive_info = [angle, speed]
    pub_data = Int32MultiArray(data=drive_info)
    motor_pub.publish(pub_data)

def callback(data):
    global usonic_data
    usonic_data = data.data

if __name__ == '__main__':
    init_node()
    time.sleep(3)

    rate = rospy.Rate(10)
    
    num = 0
    speed = 110
    stop_front = 30
    stop_back = 45
    isFront = False

    while not rospy.is_shutdown():
        while num < 3:
            if not isFront:
                if speed >= 120 and speed < 130:
                    stop_front = 60
                elif speed >= 130:
                    stop_front = 80
                if usonic_data[1] <= stop_front:
                    drive(90, 90)
                    speed += 10
                    isFront = not isFront
                    time.sleep(5)
                    for stop_cnt in range(2):
                        drive(90, 90)
                        time.sleep(0.1)
                        drive(90, 70)
                        time.sleep(0.1)
                else:
                    drive(90, speed)
            else:
                if usonic_data[4] <= stop_back:
                    drive(90, 90)
                    num += 1
                    isFront = not isFront
                    time.sleep(5)
                else:
                    drive(90, 70)
                
        rate.sleep()
    rospy.on_shutdown(exit_node)