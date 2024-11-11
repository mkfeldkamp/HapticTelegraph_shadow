import socket
import time

UDP_IP =   "localhost" # "127.0.0.1" 
UDP_PORT = 8284

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))


print("Listening for UDP packets on port %s ..." % UDP_PORT)
myTimes = [ 0, 0]

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    # myTimes = [myTimes ; [data] []]
    print("Time diff: %s   " % ( int(data.decode())- time.perf_counter_ns() ))
    # print("received message: %s  from  %s" % (data, addr))
    # print(data.decode())
