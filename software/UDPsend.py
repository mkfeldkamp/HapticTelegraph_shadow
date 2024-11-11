# Minimal working example of sending stuff over UDP
# Any IP address will work as long as you have betwork permissions
# localhost is optimized, faster than 127.0.0.1 for local inter-process communication (IPC)
import socket

UDP_IP =  "localhost" # "127.0.0.1" 
UDP_PORT = 8284
MESSAGE = "Hello, World!" # use b"Hello, World!" to encode as bytes type, or .encode() as needed

print("UDP target IP: %s" % UDP_IP)
print("UDP target port: %s" % UDP_PORT)
print("message: %s" % MESSAGE)

sock = socket.socket(socket.AF_INET, # Internet
                     socket.SOCK_DGRAM) # UDP
sock.sendto(MESSAGE.encode(), (UDP_IP, UDP_PORT))

