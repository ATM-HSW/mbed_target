# Again we import the necessary socket python module
import socket
# Here we define the UDP IP address as well as the port number that we have 
# already defined in the client python script.
UDP_IP_ADDRESS = "192.168.178.25"
UDP_PORT_NO = 10001

# declare our serverSocket upon which
# we will be listening for UDP messages
print("create socket")
serverSock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
serverSock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
# serverSock.settimeout(0.0)
# One difference is that we will have to bind our declared IP address
# and port number to our newly declared serverSock
print("bind socket")
serverSock.bind((UDP_IP_ADDRESS, UDP_PORT_NO))
while True:
    # print "recv"
#    try:
#        data, addr = serverSock.recvfrom(2)
#    except:
#        continue
    data, addr = serverSock.recvfrom(1024)
    # if not data: break
    print("Message: ", data)
