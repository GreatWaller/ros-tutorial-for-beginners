# 动手学ROS——再论Topic

之前两节已经介绍了如何创建publisher和subscriber，本节我们再补充一点关于topic的知识。

想要获取Topic中的msg，据我了解有两种方式：

- 一种是我们已经知道的，通过订阅该Topic, 注册回调函数，从中读取信息；

- 另一种自然是不需要订阅，直接从Topic取。

  这是一种有用的方式，毕竟有些时候，我们只是临时需要某个Topic中的数据。

接下来就直接看示例代码：

在《动手学ROS——自定义msg》一节中创建的custom_msg_publisher pkg中，新增wait_message.cpp:

```c++
#include "ros/ros.h"
#include "public_pkg/CustomMsg.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wait_message");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        boost::shared_ptr<public_pkg::CustomMsg const> msg;
        msg = ros::topic::waitForMessage<public_pkg::CustomMsg>("custom_topic", ros::Duration(5));

        if (msg)
        {
            ROS_INFO("%s %d", msg->name.c_str(), msg->age);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
```

请注意waitForMessage的返回类型是一个shared pointer。



测试：

首先运行publisher

```bash
$ rosrun custom_msg_publisherustom_msg_publisher_node
```

再运行wait_message_node

```bash
$ rosrun custom_msg_publisher wait_message_node 
[ INFO] [1620993369.393080200]: hello world  49
[ INFO] [1620993370.392878900]: hello world  50
[ INFO] [1620993371.398617800]: hello world  51
[ INFO] [1620993372.392693100]: hello world  52
[ INFO] [1620993373.393078300]: hello world  53
```



#### 小结