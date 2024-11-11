# Echo server program
import socket

HOST = '127.0.0.1'        # Symbolic name meaning the local host
PORT = 50007              # Arbitrary non-privileged port
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM) # TCP sockets, not UDP
s.bind((HOST, PORT))
s.listen(5)
conn, addr = s.accept()
print( 'Connected by', addr)
quit = False
while not quit:
    header = conn.recv(1)
    
    print ("server received packet header (1st byte): " )
    print (header)
    
    if header == 'Q': 
        print ("RECEIVED QUIT COMMMAND ...")
        conn.send("quiting...")
        quit = True;
        break
    else: 
        data = conn.recv(1024)  # read rest of packet
        print ("FULL PACKET:")
        print (header + data )
        conn.send(header + data)
    #if not data: break

conn.close()