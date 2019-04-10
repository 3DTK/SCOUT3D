#!/usr/bin/env python

import rospy
import smbus
import time
from scout3d_temperature.msg import TemperatureHumidity

def readTempSensor():
    # Get I2C bus
    bus = smbus.SMBus(1)

    # SI7021 address, 0x40(64)
    #		0xF5(245)	Select Relative Humidity NO HOLD master mode
    bus.write_byte(0x40, 0xF5)

    time.sleep(0.3)

    # SI7021 address, 0x40(64)
    # Read data back, 2 bytes, Humidity MSB first
    data0 = bus.read_byte(0x40)
    data1 = bus.read_byte(0x40)

    # Convert the data
    humidity = ((data0 * 256 + data1) * 125 / 65536.0) - 6

    time.sleep(0.3)

    # SI7021 address, 0x40(64)
    #		0xF3(243)	Select temperature NO HOLD master mode
    bus.write_byte(0x40, 0xF3)

    time.sleep(0.3)

    # SI7021 address, 0x40(64)
    # Read data back, 2 bytes, Temperature MSB first
    data0 = bus.read_byte(0x40)
    data1 = bus.read_byte(0x40)

    # Convert the data
    cTemp = ((data0 * 256 + data1) * 175.72 / 65536.0) - 46.85
    msg = TemperatureHumidity()
    msg.timestamp = rospy.Time.now()
    msg.temperature = cTemp
    msg.humidity = humidity
    return (msg)

def readTempCpu():
    f = open('/sys/class/thermal/thermal_zone0/temp', 'r')
    temp = float(f.readline()) / 1000.0

    msg = TemperatureHumidity()
    msg.timestamp = rospy.Time.now()
    msg.temperature = temp
    msg.humidity = 0
    return(msg)

def publishTemp():
    useSensor = rospy.get_param('~use_sensor', False)

    if (useSensor):
        pubSensor = rospy.Publisher('temperature/sensor', TemperatureHumidity, queue_size=10)
    pubCpu = rospy.Publisher('temperature/cpu', TemperatureHumidity, queue_size=10)

    rospy.init_node('temperature_node', anonymous=True)

    rate = rospy.Rate(1) # 1hz
    while not rospy.is_shutdown():
        if (useSensor):
            msg = readTempSensor()
            #TODO make source configurable
            msg.source = "sensor"
            pubSensor.publish(msg)

        msg = readTempCpu()
        #TODO make source configurable
        msg.source = "cpu"
        pubCpu.publish(msg)

        rate.sleep()

if __name__ == '__main__':
    try:
        publishTemp()
    except rospy.ROSInterruptException:
        pass
