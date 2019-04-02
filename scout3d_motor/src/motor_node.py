#!/usr/bin/env python

import rospy
import time
import threading
import subprocess
import math
from std_msgs.msg import Bool
from std_msgs.msg import Float32
from sensor_msgs.msg import JointState
from scout3d_motor.srv import MotorPositionCommand

def setZero(arg):
    global setpoint
    setpoint = float('nan')
    position = int(5000.0 * arg.position / (math.pi / 2.0))
    subprocess.check_output(["/home/scout3d/setMotorZero", str(position)])
    return []

def setPosition(arg):
    global setpoint
    setpoint = int(5000.0 * arg.position / (math.pi / 2.0))
    print('new setpoint: ' + str(arg.position))
    return []

def updatePosition():
    global setpoint
    while not rospy.is_shutdown():
        position = int(subprocess.check_output(["/home/scout3d/getMotorPosition", ""]))
        positionDeg = 90.0 * float(position) / 5000.0
        positionRad = (math.pi / 2.0) * float(position) / 5000.0

        msg = JointState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "/scanner_motor"
        msg.name = [ "base_joint" ]
        msg.position = [ positionRad ]
        msg.velocity = [ 0 ]
        msg.effort = [ 0 ]
        pubPosition.publish(msg)
        # print('position: ' + str(positionDeg) + ' (' + str(position) + ')')

        if not math.isnan(setpoint) and setpoint != position:
            # print('driving: ' + str(setpoint))
            subprocess.check_output(["/home/scout3d/setMotorPosition", str(setpoint)])

            msg = Bool()
            msg.data = False
            pubStopped.publish(msg)
        else:
            # print('deenergize')
            subprocess.check_output(["/home/scout3d/off", ""])

            msg = Bool()
            msg.data = True
            pubStopped.publish(msg)

        time.sleep(.01)

if __name__ == '__main__':
    global setpoint
    setpoint = float('nan')
    rospy.init_node('motor_node', anonymous=True)
    pubPosition = rospy.Publisher('motorPosition', JointState, queue_size=10)
    pubStopped = rospy.Publisher('motorStopped', Bool, queue_size=10)
    serPos = rospy.Service('setMotorPosition', MotorPositionCommand, setPosition)
    serZero = rospy.Service('setMotorZero', MotorPositionCommand, setZero)
    try:
        threading.Thread(target=updatePosition).start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
