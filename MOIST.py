#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os       #Library to get env variables
import sys      #Library to exit program on error
import serial   #Library for serial communication
import time     #Library for the delay
import requests #Library for HTTP requests
import json     #Library for creating JSON object
import fcntl    #Library for resource locking

FILENAME = os.path.splitext(os.path.basename(__file__))[0]
TOKEN = os.environ.get("UBIDOTS_TOKEN")
PORT = '/dev/ttyUSB0'
BAUDRATE = 9600
SERIAL_TIMEOUT = 3           #How long to wait(s) before receiving data back
SERIAL_RETRY_INTERVAL = 5    #How long to wait(s) before retrying serial connection
MAX_SERIAL_RETRY = 10        #Maximum number of retries for serial connection
HTTP_RETRY_INTERVAL = 1      #How long to wait(s) before retrying HTTP connection
MAX_HTTP_RETRY = 5           #Maximum number of retries for HTTP connection

ser = serial.Serial()
ser.baudrate = BAUDRATE
ser.port = PORT
ser.timeout = SERIAL_TIMEOUT
ser.dtr = True

serial_retry_counter = 0 #Counter retry to open serial port connection to Arduino

while serial_retry_counter < MAX_SERIAL_RETRY:
    ser.open()
    if ser.isOpen():
        try:
            fcntl.flock(ser, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except IOError:
            ser.close()
            time.sleep(SERIAL_RETRY_INTERVAL)
            serial_retry_counter += 1
        else:
            serial_retry_counter = MAX_SERIAL_RETRY

if not ser.isOpen():
    sys.exit('{0} cannot open {1}'.format(PORT, FILENAME))

ser.reset_input_buffer()
time.sleep(0.1)

def post_request(payload):
    # Creates the headers for the HTTP requests
    url = "https://things.ubidots.com/api/v1.6/devices/greenpi"
    headers = {"X-Auth-Token": TOKEN, "Content-Type": "application/json"}

    # Makes the HTTP requests
    status = 400
    attempts = 0
    while status >= 400 and attempts <= MAX_HTTP_RETRY:
        req = requests.post(url=url, headers=headers, json=payload)
        status = req.status_code
        attempts += 1
        time.sleep(HTTP_RETRY_INTERVAL)

    # Processes results
    if status >= 400:
        print("[ERROR] Could not send data after 5 attempts, please check \
            your token credentials and internet connection")
        return False

    return True

ser.write((FILENAME + '\n').encode('ascii'))
ser.flush()
time.sleep(0.1)
payload = ser.readline().decode('utf-8').rstrip()
ser.close()
if payload:
    post_request(json.loads(payload))

