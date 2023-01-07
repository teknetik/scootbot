import RPi.GPIO as GPIO
import time

GPIO.setmode(GPIO.BCM)
pinList = [26, 19, 13, 6]

for i in pinList:
    GPIO.setup(i, GPIO.OUT)
    GPIO.output(i, GPIO.HIGH)

GPIO.output(6, GPIO.LOW)
#time.sleep(10)
#GPIO.output(19, GPIO.HIGH)
