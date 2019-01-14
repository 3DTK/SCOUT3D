#!/usr/bin/env python

import rospy
import time
import threading
import subprocess
import math
from std_msgs.msg import Float32
from scout3d_motor.msg import MotorPosition
from scout3d_motor.srv import MotorPositionCommand

def setZero(arg):
    global setpoint
    setpoint = float('nan')
    position = int(5000.0 * arg.position / 90.0)
    subprocess.check_output(["/home/laser/setMotorZero", str(position)])
    return []

def setPosition(arg):
    global setpoint
    setpoint = int(5000.0 * arg.position / 90.0)
    print('new setpoint: ' + str(arg.position))
    return []

def updatePosition():
    global setpoint
    while not rospy.is_shutdown():
        position = int(subprocess.check_output(["/home/laser/getMotorPosition", ""]))
        positionDeg = 90.0 * float(position) / 5000.0

        msg = MotorPosition()
        msg.timestamp = rospy.Time.now()
        msg.position = positionDeg
        pub.publish(msg)
        # print('position: ' + str(positionDeg) + ' (' + str(position) + ')')
        if not math.isnan(setpoint):
            if setpoint != position:
                # print('driving: ' + str(setpoint))
                subprocess.check_output(["/home/laser/setMotorPosition", str(setpoint)])
            else:
                # print('deenergize')
                subprocess.check_output(["/home/laser/off", ""])
        time.sleep(.01)

if __name__ == '__main__':
    global setpoint
    setpoint = float('nan')
    rospy.init_node('motor_node', anonymous=True)
    pub = rospy.Publisher('motorPosition', MotorPosition, queue_size=10)
    serPos = rospy.Service('setMotorPosition', MotorPositionCommand, setPosition)
    serZero = rospy.Service('setMotorZero', MotorPositionCommand, setZero)
    try:
        threading.Thread(target=updatePosition).start()
        rospy.spin()
        ser.close()
    except rospy.ROSInterruptException:
        pass
