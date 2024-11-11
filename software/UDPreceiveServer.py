# Minimal working example of receiving stuff over UDP
# Any IP address will work as long as you have betwork permissions
# localhost is optimized, faster than 127.0.0.1 for local inter-process communication (IPC)

import socket
UDP_IP =   "localhost" # "127.0.0.1"  
UDP_PORT = 8284

sock = socket.socket(socket.AF_INET,    # Internet
                     socket.SOCK_DGRAM) # UDP
sock.bind((UDP_IP, UDP_PORT))

print("Listening for UDP packets on port %s ..." % UDP_PORT)

while True:
    data, addr = sock.recvfrom(1024) # buffer size is 1024 bytes
    print("received message: %s " %  data.decode())  # use data.decode()
    

    
