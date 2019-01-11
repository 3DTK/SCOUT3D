#! /usr/bin/python

import sys
import serial
import time

ser = serial.Serial('/dev/ttyUSB0', 115200)

print ser.port
print ser.baudrate
print sys.argv[1]
print sys.argv[2]
print sys.argv[3]

time.sleep(2)
ser.write(str(sys.argv[1]) + ' ' + str(sys.argv[2]) + ' ' + str(sys.argv[3]) + '\n')
while True:
	time.sleep(1)
ser.close()
