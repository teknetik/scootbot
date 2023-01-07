from pysabertooth import Sabertooth
import time

saber = Sabertooth('/dev/ttyS0', baudrate=115200, address=128, timeout=0.1)

#Right Wheel
saber.drive(1, 30)
#Left wheel
saber.drive(2, 30)

print("Sleep")
time.sleep(30)
print("stop")
saber.stop()
