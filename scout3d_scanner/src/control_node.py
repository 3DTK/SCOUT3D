#!/usr/bin/env python

import rospy
import time
import threading
import subprocess
import math
import time
import dynamic_reconfigure.client
import subprocess, shlex
import psutil,signal
from std_msgs.msg import Bool
from scout3d_scanner.srv import ScanCommand
from scout3d_motor.srv import MotorPositionCommand
from scout3d_laser.srv import LaserPowerCommand

def handleMotorStopped(arg):
    global motorStopped
    motorStopped = arg.data

def startScan(req):
    global doScan

    if (doScan):
        return []

    doScan = True

    global start_angle
    global stop_angle
    global laser0_power
    global laser1_power
    global camera_shutter
    global camera_gain

    start_angle = req.start_angle
    stop_angle = req.stop_angle
    laser0_power = req.laser0_power
    laser1_power = req.laser1_power
    camera_shutter = req.camera_shutter
    camera_gain = req.camera_gain

    return []

def processScan():
    while not rospy.is_shutdown():
        global doScan
        global motorStopped

        global start_angle
        global stop_angle
        global laser0_power
        global laser1_power
        global camera_shutter
        global camera_gain

        if (doScan):
            command = "rosbag record -b 2048 -o /home/scout3d/Bag/scout3d /camera/image_raw /camera/imu/data /camera/imu/data_raw /camera/imu/is_calibrated /camera/imu/mag /camera/imu/magnetic_field /camera/temperature/cpu /laser/imu/data /laser/imu/data_raw /laser/imu/is_calibrated /laser/imu/mag /laser/imu/magnetic_field /laser/temperature/cpu /laser/temperature/sensor /motor/encoder_raw /motor/motorPosition /motor/temperature/cpu"

            setMotorPosition = rospy.ServiceProxy('/motor/setMotorPosition', MotorPositionCommand)
            setLaserPower = rospy.ServiceProxy('/laser/setLaserPower', LaserPowerCommand)

            client = dynamic_reconfigure.client.Client('/camera/spinnaker_camera_nodelet')
            params = { 'acquisition_frame_rate' : 41, 'exposure_time' : camera_shutter * 1e6, 'gain' : camera_gain }
            config = client.update_configuration(params)

            setLaserPower(0, 0, 0)
            setMotorPosition(start_angle)
            time.sleep(1)
            motorStopped = False
            while (motorStopped == False):
                time.sleep(0.01)

            rosbag_proc = subprocess.Popen(command, shell=True)
            time.sleep(3)

            setLaserPower(0, laser0_power, laser1_power)
            setMotorPosition(stop_angle)
            time.sleep(1)
            motorStopped = False
            while (motorStopped == False):
                time.sleep(0.01)
            setLaserPower(0, 0, 0)

            try:
                p = psutil.Process(rosbag_proc.pid)
                try:
                    for sub in p.get_children(recursive=True):
                        sub.send_signal(signal.SIGINT)
                        rosbag_proc.send_signal(signal.SIGINT)
                except AttributeError:
                    for sub in p.children(recursive=True):
                        sub.send_signal(signal.SIGINT)
                        rosbag_proc.send_signal(signal.SIGINT)
            except Exception as e:
                print str(e)

            doScan = False

        time.sleep(0.01)


if __name__ == '__main__':
    global doScan
    doScan = False
    global motorStopped
    motorStopped = False

    rospy.init_node('control_node', anonymous=True)

    rospy.wait_for_service("/motor/setMotorPosition")
    rospy.wait_for_service("/laser/setLaserPower")

    rospy.Subscriber("/motor/motorStopped", Bool, handleMotorStopped)

    serviceStartScan = rospy.Service('startScan', ScanCommand, startScan)
    try:
        threading.Thread(target=processScan).start()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
