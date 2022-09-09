import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
pinList = [26, 19, 13, 6]


GPIO.setup(6, GPIO.OUT)
GPIO.output(6, GPIO.LOW)
