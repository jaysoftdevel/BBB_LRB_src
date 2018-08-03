#!/usr/bin/python

import Adafruit_BBIO.GPIO as GPIO

GPIO.setup("P8_3", GPIO.OUT)
if GPIO.input("P8_3") == 1:
	print("Setting pin P8_3 LOW")
	GPIO.output("P8_3", GPIO.LOW)
else:
	print("Setting pin P8_3 HIGH")
	GPIO.output("P8_3", GPIO.HIGH)

