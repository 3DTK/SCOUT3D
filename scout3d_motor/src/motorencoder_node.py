#!/usr/bin/env python

import rospy
import serial
import threading
from sensor_msgs.msg import JointState

def publishEncoderValues():
    direction = 0 # motor direction -1 and +1
    totalticks = 0
    lasttickvalue = 0
    while ser.readable() and not rospy.is_shutdown():
        data = ser.readline()
        ser.flushInput()
        datalist = data.split(' ')
        packet = ''
        value = 0
        status = 0
        button = 0
        counter = 0
        if len(datalist) == 5:
            try:
                packet = datalist[0]
                value = int(datalist[1])
                status = int(datalist[2])
                button = int(datalist[3])
                counter = int(datalist[4])
            except ValueError:
                continue
            if 4096 >= value >= 0:
                msg = JointState()
                msg.header.stamp = rospy.Time.now()
                msg.header.frame_id = "/encoder_raw"
                msg.name = [ "encoder_joint" ]
                msg.position = [ value ]
                msg.velocity = [ 0 ]
                msg.effort = [ 0 ]
                pub.publish(msg)
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
    pub = rospy.Publisher('encoder_raw', JointState, queue_size=10)
    try:
        ser = serial.Serial('/dev/ttyS0', 115200)
        threading.Thread(target=publishEncoderValues).start()
        rospy.spin()
        ser.close()
    except rospy.ROSInterruptException:
        pass
