#!/usr/bin/python
# -*- coding: utf-8 -*-

import time
import socket
import rospy
from std_msgs.msg import Empty,UInt8

######################tcp begining
addr = ('192.168.125.1',4044)
HOST = '192.168.125.5'

PORT=2522

BUFFER=1024

sock=socket.socket(socket.AF_INET,socket.SOCK_DGRAM)

sock.bind((HOST,PORT))

# sock.listen(5)

def callback(data):
    rospy.loginfo(rospy.get_caller_id() + "sending move_request")
    sock.sendto('move'.encode(),addr)

################ros begining
rospy.init_node('tcptalker',anonymous=0)
pub=rospy.Publisher('gocator_3200/snapshot_request',Empty,queue_size=1)
# pub_num=rospy.Publisher('gocator_3200/measure_number',UInt8,queue_size=1)
sub=rospy.Subscriber('gocator_3200/move_request',Empty,callback)

print 'i am listening'

while not rospy.is_shutdown():
    # _ ,addr=sock.accept()
    try:
        # con.settimeout(5)
        buf , _ =sock.recvfrom(BUFFER)
        print buf
        text_file = open("/home/loganshi/Documents/gocator_pcl/src/pcl_pub/results/test.txt", "a")
        text_file.write(buf)
        text_file.close()
        pub.publish()

        # s1 = buf[0]
        # s2 = buf[1]
        # num = 0
        # if s2.isdigit():
        #     num = int(s1+s2)
        # else:
        #     num = int(s1)
            
        # pub_num.publish(s1)
        # time.sleep(1)
    except socket.timeout:
        print 'time out'
    # con.send('yes i recv')

# con.close()