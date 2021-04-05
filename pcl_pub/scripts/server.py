#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import socket
import rospy
from std_msgs.msg import Empty

######################tcp begining
HOST='192.168.125.5'

PORT=2522

BUFFER=1024

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

sock.bind((HOST,PORT))

# sock.listen(5)

################ros begining
rospy.init_node('tcptalker',anonymous=0)
pub=rospy.Publisher('gocator_3200/snapshot_request',Empty,queue_size=1)

print 'i am listening'

while not rospy.is_shutdown():
    # con,addr=sock.accept()
    try:
        # con.settimeout(5)
        buf , _ =sock.recvfrom(BUFFER)
        print buf
        pub.publish()
        # time.sleep(1)
    except socket.timeout:
        print 'time out'
    # con.send('yes i recv')

# con.close()