import subprocess

uptime = subprocess.check_output(["uptime"])
uptime=uptime.decode('UTF-8').split()[0]

print(uptime)