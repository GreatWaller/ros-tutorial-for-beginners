import rospy
from sensor_msgs.msg import Image

class ImageSubscriber():
    def __init__(self) -> None:
        self.node=rospy.init_node('image_listener')
        self.rate = rospy.Rate(1)
        self.sub= rospy.Subscriber('image_topic',Image,self.callback,queue_size=1)

    def callback(self,msg:Image):
        print(msg.header.seq)
        self.rate.sleep()

if __name__ == '__main__':
    ImageSubscriber()
    rospy.spin()