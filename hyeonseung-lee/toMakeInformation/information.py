#!/usr/bin/env python
from autodrive1 import Autodrive
import rospy, time
from diagnostic_msgs.msg import DiagnosticArray

imu_data = None
information = { "speed" : 0,
                "signal": "",
                "means" : "",

                "roll" : 0,
                "pitch"  : 0,
                "yaw"   : 0,
                }
def callback(data):
    global imu_data
    imu_data = data.status[0].values

rospy.init_node('IMU_subscriber', anonymous=True)
rospy.Subscriber('/diagnostics', DiagnosticArray, callback, queue_size=1)

time.sleep(15)

while not rospy.is_shutdown():
    information["roll"] = imu_data[0].value
    information["pitch"]= imu_data[1].value
    information["yaw"] = imu_data[2].value

    information["speed"] = Autodrive.getter()

    # subscribe signal node

    time.sleep(1)


Autodrive.getter()