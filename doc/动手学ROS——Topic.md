# 动手学ROS——Topic通信

ros中的通信方式有三种:

- Topic

  类似于mq（消息队列），异步收发；分为publisher和subscriber;

- Service

  类似http请求，同步；

- Action

  基于topic，实现了一种类似于service的请求/响应通讯机制，区别在于action带有反馈机制，用来不断向客户端反馈任务的进度，并且还支持在任务中途中止运行。常用于机器人的操控。

本教程重点介绍Topic的用法，Service和Action请自行查阅官方文档，不再赘述。

> 题外话：笔者在该系列教程中只会讲解个人认为的重点，不想面面俱到；同时控制篇幅，保证每篇内容不会太长。

#### 1 定义Publisher

##### 新建pkg

```bash
$ catkin_create_pkg ros_topic std_msgs roscpp
```

注意我们选择依赖std_msgs，本节会用到此pkg中的标准类型。

##### 新建ros_topic_publisher.cpp

该示例来自 [link](http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber%28c%2B%2B%29)

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 100);

    ros::Rate loop_rate(10);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << "hello world " << count;
        msg.data = ss.str();

        ROS_INFO("%s", msg.data.c_str());

        chatter_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
```

publisher向名为chatter的topic中发布了类型为std_msgs::String的消息，频率为10Hz。 

##### 修改CMakeLists.txt

添加

```cmake
add_executable(publisher src/ros_topic_publisher.cpp)
target_link_libraries(publisher ${catkin_LIBRARIES})
```

##### 编译后执行

```bash
$ rosrun ros_topic publisher
```

输出

```bash
[ INFO] [1620976290.614303700]: hello world 0
[ INFO] [1620976290.714394100]: hello world 1
[ INFO] [1620976290.814635100]: hello world 2
[ INFO] [1620976290.914424900]: hello world 3
[ INFO] [1620976291.014889000]: hello world 4
[ INFO] [1620976291.114499100]: hello world 5
```

#### 2 定义Subscriber

##### 新建ros_topic_subscriber.cpp

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

void chatterCallback(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO("I heard: [%s]", msg->data.c_str());
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "subscriber");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe("chatter", 100, chatterCallback);

    ros::spin();
    return 0;
}
```

需要指出的是，subscribe的Topic名称“chatter”需要与publisher中的名称保持一致。后面讲到namespace时我会再提到名称的一些注意事项。

subscribe时，需要传进去一个回调函数，在回调里进行实际的业务处理。

##### 修改CMakeLists.txt

添加

```
add_executable(subscriber src/ros_topic_subscriber.cpp)
target_link_libraries(subscriber ${catkin_LIBRARIES})
```

#### 3 测试

编译成功后，启动roscore, 再打开两个终端：

- subscriber

先运行subscriber, 这样publihser一旦开始发送消息，subscriber就可以接收到

```bash
$ rosrun ros_topic subscriber
```

- publisher

```bash
$ rosrun ros_topic publisher
[ INFO] [1620978105.696939600]: hello world 0
[ INFO] [1620978105.796903900]: hello world 1
[ INFO] [1620978105.896892500]: hello world 2
[ INFO] [1620978105.996940000]: hello world 3
[ INFO] [1620978106.097085400]: hello world 4
[ INFO] [1620978106.197161200]: hello world 5
```

subscriber日志

```bash
$ rosrun ros_topic subscriber 
[ INFO] [1620978105.897826700]: I heard: [hello world 2]
[ INFO] [1620978105.997705500]: I heard: [hello world 3]
[ INFO] [1620978106.097902800]: I heard: [hello world 4]
[ INFO] [1620978106.197956000]: I heard: [hello world 5]
```

请留意一个细节，虽然subscriber是先启动的，但是并未收到全部的消息。这样的结果在平时的使用中不会造成什么问题，一般我们都是需要接收最新消息，但如果你的应用需要保证每次消息都必须接收，这就会成为一个隐患。



#### 4 小结

本节简要介绍了topic的使用过程，topic 类型为自带的std_msgs::String,  接下来一起看看如何自定义msg类型，这也是开发中最常用到的.

