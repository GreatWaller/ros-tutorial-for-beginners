
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


if __name__ == '__main__':

    pub = rospy.Publisher('image_topic', Image, queue_size=1)
    rospy.init_node('image_topic_publisher_node', anonymous=True)
    rate = rospy.Rate(5)  # 5hz

    bridge = CvBridge()
    i = 0
    while not rospy.is_shutdown():
        try:
            # read a frame
            img = cv2.imread("test/image.jpg")
            msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")
            # msg.header.seq = i
            print(msg.header.seq)
            pub.publish(msg)
            # i += 1
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
