

# 动手学ROS——roslaunch

前几篇文章在启动ros node的时候使用的是rosrun命令，但每次启动之前都需要单独启动roscore，显得有些繁琐。其实ros还提供了一种运行ros node的方式：roslaunch。

roslaunch需要一个.launch的xml文件，启动时会检查系统是否已经启动了roscore，如果没有，会自动启动。

**roslaunch file最根本的作用是维护参数，同时还可以管理多节点。**

#### 如何读取参数

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

​		- 可修改：一方面可通过roslaunch命令行传入，另一方面可通过<include> 标签从上层launch 文件传入；

node 标签表示具体要启动的ros节点，pkg 为package的名称，type为可执行文件的名字（CMakeLists.txt中add_executable的目标名称）。在launch文件中可启动相关的多个node，这也是roslaunch的优势之一。

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

    <rosparam file="$(find launch_file_tutorial)/config/config.yaml" command="load" />

    <node name="publisher" pkg="launch_file_tutorial" type="launch_publisher" output="screen"/>
</launch>
```

rosparam 通过 load 命令读取 config.yaml 文件中的配置项，保存在参数服务器中。通过yaml文件，一次可以加载大量配置，在项目中应用广泛。



#### 测试

启动方式：

```bash
roslaunch launch_file_tutorial single_publisher.launch
```

```bash
SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.10
 * /topic_name: messageA

NODES
  /
    publisher (launch_file_tutorial/launch_publisher)

auto-starting new master
process[master]: started with pid [20526]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 2193b2e8-b70b-11eb-b722-00155d817914
process[rosout-1]: started with pid [20539]
started core service [/rosout]
process[publisher-2]: started with pid [20546]
[ INFO] [1621254404.337571500]: /messageA 0
[ INFO] [1621254404.837819200]: /messageA 1
[ INFO] [1621254405.337715800]: /messageA 2
[ INFO] [1621254405.837830900]: /messageA 3
[ INFO] [1621254406.337969100]: /messageA 4
[ INFO] [1621254406.837725600]: /messageA 5
[ INFO] [1621254407.337923000]: /messageA 6
```

可以看到，启动节点前，Parameters中已经有了/topic_name。

最后，刚才提到arg标签可以修改参数值，我们来演示一下：

```
$ roslaunch launch_file_tutorial single_publisher.launch topic_name:=hello
SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.10
 * /topic_name: hello

NODES
  /
    publisher (launch_file_tutorial/launch_publisher)

auto-starting new master
process[master]: started with pid [20688]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to faf18984-b70b-11eb-a321-00155d817914
process[rosout-1]: started with pid [20701]
started core service [/rosout]
process[publisher-2]: started with pid [20708]
[ INFO] [1621254768.669922500]: /hello 0
[ INFO] [1621254769.170149900]: /hello 1
[ INFO] [1621254769.670146100]: /hello 2
[ INFO] [1621254770.170130100]: /hello 3
[ INFO] [1621254770.670159700]: /hello 4
```

可见，topic_name的值已经从messageA 更改为 hello。通过在roslaunch命令最后附加:=的赋值操作，就可以从外部传入参数值，非常有用。

#### launch file 的层级

实际的项目中，一次性要启动许多node，同时每个node 需要用自己的launch file管理参数，因此，从最外层的launch file启动其它launch file是有必要的。我们可通过include 标签嵌套launch file来达到此目的。

```xml
<?xml version="1.0"?>
<launch>
    <arg name="topic_name" default="messageB" />
    <include file="$(find launch_file_tutorial)/launch/single_publisher.launch" >
        <arg name="topic_name" value="$(arg topic_name)" />
    </include>
</launch>
```

我们更改了topic_name的默认值，打印的结果也显示更改成功。

```bash
$ roslaunch launch_file_tutorial include.launch 
process[publisher-2]: started with pid [21239]
[ INFO] [1621256310.264659400]: /messageB 0
[ INFO] [1621256310.764872400]: /messageB 1
[ INFO] [1621256311.265068400]: /messageB 2
[ INFO] [1621256311.764930900]: /messageB 3
[ INFO] [1621256312.264859000]: /messageB 4
```



#### 小结

1. 通过roslaunch启动，可自动运行roscore;
2. launch file 主要作用是管理parameters;
3. 介绍了基本的传参手段；
4. launch file可逐级嵌套。

下一节，我们将聊一聊ros中的namespace。
