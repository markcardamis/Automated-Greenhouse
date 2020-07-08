#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os       #Library to get env variables
import serial   #Library for serial communication
import time     #Library for the delay
import requests #Library for HTTP requests
import json     #Library for creating JSON object

TOKEN = os.environ.get("UBIDOTS_TOKEN")
ser = serial.Serial('/dev/ttyUSB0', 9600,timeout=1)
ser.reset_input_buffer()

time.sleep(4)   #Wait for the arduino to be ready

Counter=0

def post_request(payload):
    # Creates the headers for the HTTP requests
    url = "https://things.ubidots.com/api/v1.6/devices/greenpi"
    headers = {"X-Auth-Token": TOKEN, "Content-Type": "application/json"}

    # Makes the HTTP requests
    status = 400
    attempts = 0
    while status >= 400 and attempts <= 5:
        req = requests.post(url=url, headers=headers, json=payload)
        status = req.status_code
        attempts += 1
        time.sleep(1)

    # Processes results
    if status >= 400:
        print("[ERROR] Could not send data after 5 attempts, please check \
            your token credentials and internet connection")
        return False

    return True



while True:
    ser.reset_input_buffer()
    ser.write(b'GET')
    ser.flush()
    time.sleep(10)
    payload = ser.readline().decode('utf-8').rstrip()
    print(payload)
    if payload:
        post_request(json.loads(payload))

