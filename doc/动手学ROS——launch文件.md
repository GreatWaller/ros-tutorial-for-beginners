

# roslaunch 文件

前几篇文章在启动ros node的时候使用的是rosrun命令，但每次启动之前都需要单独启动roscore，显得有些繁琐。其实ros还提供了一种运行ros node的方式：roslaunch。

roslaunch需要一个.launch的xml文件，启动时会检查系统是否已经启动了roscore，如果没有，会自动启动。

**roslaunch file最根本的作用是维护参数，同时还可以管理多节点。**

#### 读取参数

我们首先介绍代码中如何去取参数。新建一个publisher, Topic的名称是从参数服务器中取得的：

```c++
#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
	// 获取参数
    std::string topicName;
    n.param<std::string>("topic_name", topicName, "message");

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>(topicName, 100);

    ros::Rate loop_rate(2);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        // 发送的消息中包含namespace
        std::string ns = n.getNamespace().compare("/") == 0 ? "" : "/";
        ss << ns << "/" << topicName << " " << count;
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

获取参数的方法参见上述代码，该示例会打印出的信息会包含namespace, 方便后续观察。

#### launch文件的传参方式

这里提供两种方式，其结果都是传入key为topic_name的参数。

- param 标签

```xml
<?xml version="1.0"?>
<launch>
    <arg name="topic_name" default="messageA" />

    <param name="topic_name" value="$(arg topic_name)"/>

    <node name="publisher" pkg="launch_file_tutorial" type="launch_publisher" output="screen"/>
</launch>
```

​	建议通过arg标签管理参数，优点有2：

​		- 可设置默认值；

​		- 可修改：一方面可通过roslaunch命令行传入，另一方面可通过<include> 标签从上层launch 文件传入；为了使文章主题明确，本文不会具体演示如何做该操作，有心的读者可自行查阅资料。

- rosparam 标签

rosparam可读取yaml格式的配置文件，十分方便。

新建config/config.yaml

```yaml
topic_name: messageA
```

修改launch文件

```xml
<?xml version="1.0"?>
<launch>
    <arg name="topic_name" default="messageA" />

    <!-- <param name="topic_name" value="$(arg topic_name)"/> -->
    <rosparam file="$(find launch_file_tutorial)/config/config.yaml" command="load" />

    <node name="publisher" pkg="launch_file_tutorial" type="launch_publisher" output="screen"/>
</launch>
```

rosparam 通过 load 命令读取 config.yaml 文件中的配置项，保存在参数服务器中。通过yaml文件，一次可以加载大量配置，在项目中应用广泛。



#### 测试

