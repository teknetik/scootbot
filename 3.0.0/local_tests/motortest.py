import serial 
import time
from datetime import datetime
import secrets
from influxdb_client import InfluxDBClient, Point, WritePrecision
from influxdb_client.client.write_api import SYNCHRONOUS
import re

# You can generate an API token from the "API Tokens Tab" in the UI
token = secrets.token
org = "draper.carl@gmail.com"
url = "https://eu-central-1-1.aws.cloud2.influxdata.com"
bucket = "motors"

def writeFlux(device, data):
	with InfluxDBClient(url=url, token=token, org=org) as client:
		write_api = client.write_api(write_options=SYNCHRONOUS)

		point = Point("speed") \
			.tag("device", device) \
			.field("c", data) \
			.time(datetime.utcnow(), WritePrecision.NS)

		write_api.write(bucket, org, point)

ser = serial.Serial('/dev/ttyUSB0', 115200, timeout=1) 

ser.write(str.encode('1,START\r'))   
ser.write(str.encode('2,START\r')) 
time.sleep(1)

speed = 20
ser.write(str.encode('1,S' + str(speed) + '\r'))
time.sleep(0.25)
ser.write(str.encode('2,S' + str(speed) + '\r')) 

waitPeriod = 600*4
for i in range(0,waitPeriod):
    ser.write(str.encode('1,GETS\r'))
    x = ser.readline()
    x = x.decode('utf-8')
    m = re.search(r'1,[sS](\d+)', x)
    writeFlux('R-Wheel', int(m.group(1)))
    ser.write(str.encode('2,GETS\r'))
    x = ser.readline()
    x = x.decode('utf-8')
    print(x)
    m = re.search(r'2,[sS](\d+)', x)
    writeFlux('L-Wheel', int(m.group(1)))
    time.sleep(0.25)

ser.write(str.encode('1,S0\r'))
time.sleep(0.25)
ser.write(str.encode('2,S0\r'))

ser.close() 