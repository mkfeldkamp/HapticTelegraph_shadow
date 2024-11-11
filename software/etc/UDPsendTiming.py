import socket
import time

UDP_IP =  "localhost" # "127.0.0.1" #
UDP_PORT = 8284
MESSAGE = b"Hello, World!"

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
# sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

# sock.sendto("test2".encode(), (UDP_IP, UDP_PORT))

for i in range(9):
    sock.sendto(str( time.perf_counter_ns() ).encode(), (UDP_IP, UDP_PORT))