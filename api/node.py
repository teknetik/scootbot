from __future__ import print_function
from flask import Flask, render_template, request, redirect, make_response, Response, send_from_directory, send_file
import time
import json
import roslibpy
import subprocess
from vedirect import Vedirect
import psutil
from hurry.filesize import size

app = Flask(__name__)





client = roslibpy.Ros(host='10.0.0.3', port=9090)
client.run()

@app.route("/")
def main():
    return Response("{'Data':'Hello World!'}", status=200, mimetype='application/json')

@app.route("/topics")
def get_topics():
    print('Is ROS connected?', client.is_connected)
    topics = roslibpy.Ros.get_topics(client)
    return Response(json.dumps(topics), status=200, mimetype='application/json')

@app.route("/nodes")
def get_nodes():
    print('Is ROS connected?', client.is_connected)
    nodes = roslibpy.Ros.get_nodes(client)
    return Response(json.dumps(nodes), status=200, mimetype='application/json')

@app.route("/health")
def wifi():
    wifi = subprocess.check_output(["iwconfig", "wlan0"])
    wifi=wifi.decode('UTF-8').split()
    wifi=wifi[25].split('=')
    wifi=wifi[1].split('/')

    uptime = subprocess.check_output(["uptime"])
    uptime=uptime.decode('UTF-8').split()[2]

    # https://github.com/karioja/vedirect
    ve = Vedirect('/dev/ttyUSB0', 60)
    battery = ve.read_data_single()

    memUsage={}
    memUsage['used']=str(size(psutil.virtual_memory().used))
    memUsage['free']=str(size(psutil.virtual_memory().available))

    results={}
    results['signal_strength']=wifi[0]
    results['uptime']=uptime
    results['battery']=battery
    results['memUsage']=memUsage

    

    return Response(json.dumps(results), status=200, mimetype='application/json')

if __name__ == "__main__":
    app.run(host='0.0.0.0', debug=True, port=5001)




    

