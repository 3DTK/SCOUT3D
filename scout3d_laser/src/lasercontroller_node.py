#!/usr/bin/env python

import rospy
import serial
import threading
import std_msgs
from scout3d_laser.srv import LaserPowerCommand
from decimal import *

def setLaserPower(req):
    # check if values in range
    if not (req.green_power >= 0 and req.green_power <= 1):
        green_power = 0
    else:
        green_power = round(req.green_power, 3)
    if not (req.blue_power >= 0 and req.blue_power <= 1):
        blue_power = 0
    else:
        blue_power = round(req.blue_power, 3)
    if (req.laser_mode is not 0) and (req.laser_mode is not 1):
        laser_mode = 0
    else:
        laser_mode = req.laser_mode

    # send to arduino
    ser.write(str(laser_mode) + ' ' + str(green_power) + ' ' + str(blue_power) +'\n')
    return []

def publishLaserMessage():
    while ser.readable() and not rospy.is_shutdown():
        pub.publish(data=ser.read())
    return []

if __name__ == '__main__':
    rospy.init_node('lasercontroller_node', anonymous=True)
    rospy.Service('setLaserPower', LaserPowerCommand, setLaserPower)
    pub = rospy.Publisher('laserMessage', std_msgs.msg.String, queue_size=10)
    try:
        ser = serial.Serial('/dev/ttyUSB0', 115200)
        threading.Thread(target=publishLaserMessage).start()
        rospy.spin()
        ser.close()
    except rospy.ROSInterruptException:
        pass
