#!/usr/bin/env python
# -*- coding: utf-8 -*-

import serial   #Library for serial communication
import time     #Library for the delay


ser = serial.Serial('/dev/ttyUSB0', 9600,timeout=1)
ser.flushInput()

time.sleep(4)   #Wait for the arduino to be ready

Counter=0

while True:
    if Counter == 10:
        Counter = 0
        ser.write(b"TPINT!\n")
        line = ser.readline().decode('utf-8').rstrip()
        print(line)

    Counter = Counter +1
    time.sleep(1)
