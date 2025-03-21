#!/usr/bin/env python
# Authors: Junior Costa de Jesus #

import rospy
from sensor_msgs.msg import LaserScan
import time

def scan_callback(msg):
    ang_inc = msg.angle_increment
    ranges = msg.ranges
    angle = 0
    print("***************************")
    print(msg.scan_time)
    for i in range(4):
        angle = 3.1415926 / 2.0 * i
        print(angle * 180 / 3.1415926, " ", ranges[int(angle / ang_inc)])

def shutdown():
    print("shutdown")

if __name__ == '__main__':
    rospy.init_node('laser_test')
    sub = rospy.Subscriber('/burger2/scan', LaserScan, scan_callback)
    rospy.on_shutdown(shutdown)
    while (not rospy.is_shutdown()):
        time.sleep(1)
    rospy.spin()
