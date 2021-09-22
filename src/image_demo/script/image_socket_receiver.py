#!/usr/bin/env python

import rospy
import socket

from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import struct

HOST = 'localhost'
PORT = 2021

def recvall(sock, count):
    buf = b''
    while count:
        newbuf = sock.recv(count)
        if not newbuf: return None
        buf += newbuf
        count -= len(newbuf)
    return buf

if __name__ == '__main__':
    try:
        pub = rospy.Publisher('/robot_scene/stream', Image, queue_size=10)
        rospy.init_node('image_receiver_node', anonymous=True)
        rate = rospy.Rate(10)  # 10hz

        # create an INET, STREAMing socket
        serversocket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        # bind the socket to a public host, and a well-known port
        serversocket.bind((HOST, PORT))

        serversocket.listen(5)

        bridge = CvBridge()
        while True:
            (clientsocket, address) = serversocket.accept()

            payload_size = struct.calcsize(">i")

            packed_msg_size=clientsocket.recv(payload_size)
            # packed_msg_size = data[:payload_size]
            msg_size = struct.unpack(">i", packed_msg_size)[0]
            stringData = recvall(clientsocket, msg_size)
 
            data = np.fromstring(stringData, dtype='uint8')

            img_np = cv2.imdecode(data, cv2.IMREAD_COLOR)
            msg = bridge.cv2_to_imgmsg(img_np, encoding="bgr8")

            pub.publish(msg)
            clientsocket.close()
            rate.sleep()


    except rospy.ROSInterruptException:
        pass
