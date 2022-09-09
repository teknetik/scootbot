from Phidget22.Phidget import *
from Phidget22.Devices.Encoder import *
import time

def onPositionChange(self, positionChange, timeChange, indexTriggered):
    print("PositionChange: " + str(positionChange))
    print("TimeChange: " + str(timeChange))
    print("IndexTriggered: " + str(indexTriggered))
    print("getPosition: " + str(self.getPosition()))

def main():
	encoder0 = Encoder()

	encoder0.setChannel(0)

	encoder0.setOnPositionChangeHandler(onPositionChange)

	encoder0.openWaitForAttachment(5000)

	try:
		input("Press Enter to Stop\n")
	except (Exception, KeyboardInterrupt):
		pass

	encoder0.close()

main()
