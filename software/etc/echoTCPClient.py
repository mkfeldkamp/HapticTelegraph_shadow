# Echo client program
import socket

HOST = '127.0.0.1'        # The remote host
PORT = 50007              # The same port as used by the server
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP sockets, NOT UDP
s.connect((HOST, PORT))
for i in range(10):
    s.send("Hello, world\r\n")
    data = s.recv(1024)

s.send('Q')    
s.close()
data = s.recv(1024)
print ('Received', repr(data))