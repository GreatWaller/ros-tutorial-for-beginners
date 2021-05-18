# 动手学ROS（7）：精讲多线程之MultiThreadedSpinner

https://levelup.gitconnected.com/ros-spinning-threading-queuing-aac9c0a793f

http://wiki.ros.org/actionlib_tutorials/Tutorials/SimpleActionClient%28Threaded%29



> 目录
>
> - 问题引入：订阅多个Topic，一个Spinner thread，Callback顺序调用
> - 订阅多个Topic, 一个Callback queue, 多个Spinner thread
> - 订阅一个Topic, 并行处理Callback queue
> - 订阅多个Topic，每个Subscriber一个Callback queue



#### 单线程的问题



Subscriber代码如下：

```c++
#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"
void CallbackA(const std_msgs::String::ConstPtr &msg)
{
    std::this_thread::sleep_for(std::chrono::seconds(2));
    ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
void CallbackB(const std_msgs::String::ConstPtr &msg)
{
    ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "listener");
    ros::NodeHandle n;
    ros::Subscriber sub_b = n.subscribe("B/message", 1, CallbackB);

    ros::Subscriber sub_a = n.subscribe("A/message", 1, CallbackA);
    ros::spin();

    return 0;
}
```

其中CallbackA中停顿了2s, 但结果显示，CallbackB也跟着变成2s的调用频率。日志如下（注意打印时间）：

```
[ INFO] [1621326729.752814500]:  I heard: [/A/message 1]
[ INFO] [1621326729.754280300]:  I heard: [/B/message 4]
[ INFO] [1621326731.754527800]:  I heard: [/A/message 5]
[ INFO] [1621326731.754622700]:  I heard: [/B/message 8]
[ INFO] [1621326733.754884500]:  I heard: [/A/message 9]
[ INFO] [1621326733.755030400]:  I heard: [/B/message 12]
```

这就说明了单线程的不足，不管有多少个Subscriber，节点都只能顺序执行回调，这在某些时候是不能忍受的，因此，多线程有了用武之地。



#### 多线程



```c++
#include <sstream>
#include "ros/ros.h"
#include "std_msgs/String.h"
int main(int argc, char **argv) {
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub =     
  n.advertise<std_msgs::String("chatter", 1000);
  int chatter_count = 0;
  ros::Timer timer = n.createTimer(ros::Duration(0.01),
  [&](const ros::TimerEvent&) {
    std_msgs::String msg;
    std::stringstream ss;
    ss << "chatter messages: " << chatter_count;
    msg.data = ss.str();
    ROS_INFO("%s", msg.data.c_str());
    chatter_pub.publish(msg);
    chatter_count++;
  });
  
  ros::spin();
  return 0;
}
```







```c++
void ChatterCallback(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
  std::this_thread::sleep_for(0.02s);
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::SubscribeOptions ops;
  ops.template init<std_msgs::String>("chatter", 1, ChatterCallback);
  ops.allow_concurrent_callbacks = true;
  ros::Subscriber sub1 = n.subscribe(ops);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}
```





![img](https://miro.medium.com/max/2004/1*zEBn_D7IOZlTWf-bnr3nuA.png)



![img](https://miro.medium.com/max/2036/1*KyskEVrKEWY3PN9mDNvJXQ.png)



```c++
#include <thread>
#include <ros/callback_queue.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
void CallbackA(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
void CallbackB(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub_b = n.subscribe("MessageB", 1, CallbackB);
  
  ros::NodeHandle n_a;
  ros::CallbackQueue callback_queue_a;
  n_a.setCallbackQueue(&callback_queue_a);
  ros::Subscriber sub_a = n_a.subscribe("MessageA", 1, CallbackA);
  std::thread spinner_thread_a([&callback_queue_a]() {
    ros::SingleThreadedSpinner spinner_a;
    spinner_a.spin(&callback_queue_a);
  });
  ros::spin();
  spinner_thread_a.join();
  return 0;
}
```





示例

https://github.com/wenglihong/wlh_ros_demo/blob/master/multi_thread_demo/src/multi_thread_listener2.cpp