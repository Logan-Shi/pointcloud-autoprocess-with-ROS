#!/usr/bin/python

import time
import socket
HOST='192.168.1.5'
PORT=65432
BUFFER=1024

while 1:
    soc=socket.socket(socket.AF_INET,socket.SOCK_STREAM)
    soc.connect((HOST,PORT))
    time.sleep(1)
    soc.send('hello ros')
    buf=soc.recv(BUFFER)
    print(buf)
    soc.close()