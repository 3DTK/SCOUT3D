#!/usr/bin/env python

import rospy
import serial
import threading
from std_msgs.msg import UInt16

def publishEncoderValues():
    direction = 0 # motor direction -1 and +1
    totalticks = 0
    lasttickvalue = 0
    while ser.readable() and not rospy.is_shutdown():
        data = ser.readline()
        ser.flushInput()
        datalist = data.split(',')
        value = 0
        deg = 0.0
        if len(datalist) == 3:
            try:
                value = int(datalist[0])
                deg = float(datalist[2])
            except ValueError:
                continue
            if 4095 >= value >= 0 and 360 >= deg >= 0:
                pub.publish(value)
                print(str(value))
                lasttickvalue = value
                if direction > 0:
                    diff = 0
                    if lasttickvalue > value: # overflow
                        diff = (4095-lasttickvalue) + value
                    if lasttickvalue < value: # non overflow
                        diff = value - lasttickvalue
                    totalticks += diff
                if direction < 0:
                    diff = 0
                    if lasttickvalue < value: # overflow
                        diff = lasttickvalue + (4095-value)
                    if lasttickvalue > value: # non overflow
                        diff = lasttickvalue - value
                    totalticks -= diff


if __name__ == '__main__':
    rospy.init_node('motorencoder_node', anonymous=True)
    pub = rospy.Publisher('encoder_raw', UInt16, queue_size=10)
    try:
        ser = serial.Serial('/dev/ttyS0', 115200)
        threading.Thread(target=publishEncoderValues).start()
        rospy.spin()
        ser.close()
    except rospy.ROSInterruptException:
        pass
