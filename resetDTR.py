#!/usr/bin/env python
# -*- coding: utf-8 -*-

import time     #Library for the delay
import termios

time.sleep(4)   #Wait for Arduino to boot
port = '/dev/ttyUSB0'
f = open(port)
attrs = termios.tcgetattr(f)
attrs[2] = attrs[2] & ~termios.HUPCL
termios.tcsetattr(f, termios.TCSAFLUSH, attrs)
f.close()
