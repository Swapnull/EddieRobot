#!/usr/bin/env python

import sys
from socket import socket, AF_INET, SOCK_DGRAM
from keypress import Keypress

SERVER_IP   = '192.168.1.227' #IPAddress of the computer on the eddie bot. 
PORT_NUMBER = 5000 
SIZE = 1024
print ("Test client sending packets to IP {0}, via port {1}\n".format(SERVER_IP, PORT_NUMBER))

mySocket = socket( AF_INET, SOCK_DGRAM )
keys = Keypress()

while True:
	key = keys.wait_for_input()
	if key != None:
		print "sending", key
		mySocket.sendto(key,(SERVER_IP,PORT_NUMBER))
sys.exit()