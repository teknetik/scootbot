import socket
import ssl
import time

host='localhost'
port=8443

def testTLSver(host, port):
    ctx = ssl.create_default_context()
    ctx.check_hostname = False
    ctx.verify_mode = ssl.CERT_NONE

    with socket.create_connection((host, port)) as sock:
        with ctx.wrap_socket(sock, server_hostname=host) as ssock:
            return ssock.version(), ssock.getpeername()


for i in range(0, 30):
    try:
        print(testTLSver(host, port))
    except Exception as e:
        print(e)
    time.sleep(1)