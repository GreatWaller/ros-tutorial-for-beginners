# 动手学ROS（14）：Topic之queue size--获取最新消息

很早之前在《动手学ROS（2）：Topic通信》中已经介绍过Topic的发送与订阅方法，本节来补充一个重要的知识点：

**如何接收最新消息**。

先说结论：**把publisher和subscriber的queue_size均设置为1**。

我们知道，ROS给提供了队列的缓存功能，通过queue_size的数值来决定缓存队列的大小。[ros文档](http://wiki.ros.org/roscpp/Overview/Publishers%20and%20Subscribers)中给出的解释是:

> `queue_size` [required]
>
> - Publisher: This is the size of the outgoing message queue. If you are publishing faster than roscpp can send the messages over the wire, roscpp will start dropping old messages. A value of 0 here means an infinite queue, which can be dangerous. See the [rospy documentation on choosing a good queue_size](http://wiki.ros.org/rospy/Overview/Publishers and Subscribers#Choosing_a_good_queue_size) for more information.
> - Subscirber: This is the incoming message queue size roscpp will use for your callback. If messages are arriving too fast and you are unable to keep up, roscpp will start throwing away messages. A value of 0 here means an infinite queue, which can be dangerous

简单总结一下：

- 队列总会保留最新的几条消息
- 没事儿别设置为0

其实这些信息就足够了，我们把消息发送与接收端的队列大小都设置为1，那就能保证总是接收的最新消息；当然如果我们不能决定发送方，那接收方设置为1也能获得足够的实效性。

下面我们就做几个实验。首先给出简单的Publisher和Subscriber，之前我们已经看过C++版本的了，现在就用python来演示，Publisher每秒发送5条消息，Subscriber每秒处理一次。

Publisher:

```python
import rospy
from std_msgs.msg import String

if __name__ == '__main__':
    rospy.init_node('simple_talker')
    rate = rospy.Rate(5)

    pub = rospy.Publisher('simple_talker', String, queue_size=1)
    i = 0
    while not rospy.is_shutdown():
        msg = String(str(i))
        pub.publish(msg)
        print(msg)
        i += 1
        rate.sleep()
```

Subscriber:

```python
import rospy
from std_msgs.msg import String

class SimpleSubscriber():
    def __init__(self) -> None:
        self.node=rospy.init_node('simple_listener')
        self.rate = rospy.Rate(1)
        self.sub= rospy.Subscriber('simple_talker',String,self.callback,queue_size=1)

    def callback(self,msg):
        print(msg)
        self.rate.sleep()

if __name__ == '__main__':
    SimpleSubscriber()
    rospy.spin()
```

条件准备完毕，接下来我们就分情况测试一下。

##### 测试1

- Publisher: 1

- Subscriber: 1

```bash
$ python3 simple_publisher.py 
data: "0"
data: "1"
data: "2"
data: "3"
data: "4"
data: "5"
data: "6"
data: "7"
data: "8"
data: "9"
data: "10"
data: "11"
```

```bash
$ python3 simple_subscriber.py 
data: "1"
data: "6"
data: "11"
```

可见，该设置条件下，subscriber收到的消息以5递增，总是最新的一条。

##### 测试2

- Publisher: 5或10
- Subscriber: 1

结果同测试1。可以看出，subciber的queue size 更有决定性作用。

##### 测试3

- Publisher: 1

- Subscriber: 5

Publisher的日志与测试1一样，这里只给出subsciber的日志：

```bash
$ python3 simple_subscriber.py 
data: "1"
data: "2"
data: "3"
data: "4"
data: "5"
data: "6"
data: "22"
data: "23"
data: "24"
data: "25"
data: "26"
data: "47"
data: "48"
data: "49"
data: "50"
data: "51"
```

这里其实有点让人出乎意料的，在收到第一条消息1之后，是连续5个这样下去，从现象看，也就是**在subscriber的队列用尽时，连续填充5条消息，等待再次用尽，如此循环**。不是实时更新的。这点一定要注意。

> 有兴趣的读者可以看看ros的源码，笔者没看过源码，在此仅做一个猜测：有一个队列A是在实时更新的，push进最新的一条消息，pop出最早的消息，而调用callbak时也有一个队列B，copy出A中的全部消息，在用尽后再去copy一次。因此就会有如此现象，有了解真实情况的读者请留言，在此先行谢过。

##### 测试4

- Publisher: 5

- Subscriber: 5

结果同测试3。可见，我们更应该关注suscriber的queue size。

#### 小结

- **subscriber的queue size 更重要**；
- **subscriber的队列不是逐条实时更新的，是按队列长度更新的**；

- **接收最新消息的方法为：将publisher和subscriber的queue_size均设置为1**。这样更保险一些。

最后，本文还是有很多未涵盖的地方。比如publisher的queue size是在什么情况下起作用的（读者可以把rate调大一些，如10000，看看subsciber的接收情况，请自行测试）；又比如，在某些特定的情况下，需要保证消息不丢失，该如何权衡rate和queue size。这些都是需要在实践中总结的。



