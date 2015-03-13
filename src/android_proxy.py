#!/usr/bin/env python

import roslib; roslib.load_manifest("ptu46_teleop")
from socket import *
from time import ctime
import struct
import rospy
from ptu46_teleop.msg import AndroidControl

pub =  rospy.Publisher('/androidcontrol', AndroidControl)
rospy.init_node('android_proxy')

HOST = ''
PORT = 4413
BUFSIZ = 1024
ADDR = (HOST, PORT)

tcpSerSocket = socket(AF_INET, SOCK_STREAM)
tcpSerSocket.bind(ADDR)
tcpSerSocket.listen(5)

try:
    while True:
        print 'waiting for connect ...'
        tcpCliSock, addr = tcpSerSocket.accept()
        print '...connected from:', addr
    
        while True:
            # it stop here until recv message
            data = tcpCliSock.recv(BUFSIZ)
            if not data:
                tcpCliSock.close()
                print 'close client'
                break
            try:
                key = struct.unpack('b', data)[0]
                pub.publish(key)
            except Exception,e:
                pass
            #print key
            #tcpCliSock.send('[%s] %s'%(ctime(), data))
            #tcpCliSock.close()

except KeyboardInterrupt, EOFError:
    try:
        tcpCliSock.fileno()
        tcpCliSock.close()
        print '\nclose client in except'
    except:
        pass
    tcpSerSocket.close()
    print '\nclose server'
