# 动手学ROS（10）：命名空间之私有空间

在《动手学ROS（6）：命名空间之group与remap》中简单提到过相对命名空间，所谓相对，表示即可以相对于某个特定的namespace，也可以**相对于节点自己**。

本节主要介绍相对于节点自身的情况，我们把它称作**私有空间**。



我们以读写参数为例来对比其使用方法：

#### 相对空间

launch file:

```xml
<launch>
    <arg name="topic_name" default="messageA" />
    <param name="topic_name" value="$(arg topic_name)"/>
    <node name="publisher" pkg="launch_file_tutorial" type="launch_publisher" output="screen"/>
</launch>
```

获取参数：

```cpp
ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    std::string topicName;
    n.param<std::string>("topic_name", topicName, "message");
```

如此可以得到topic_name为messageA。

#### 私有空间

所谓私有，就是相对于节点自身。我们可以这样写launch file：

```xml
<launch>
    <arg name="topic_name" default="messageC" />

    <node name="publisher" pkg="launch_file_tutorial" type="launch_publisher" output="screen">
        <param name="topic_name" value="$(arg topic_name)"/>
    </node>
</launch>
```

值得注意的是，我们把param标签嵌套在node标签内部，这样导致的结果是参数topic_name多了一个namespace, 即/pubisher/messageC, 我们可以看测试日志：

```bash
$ roslaunch launch_file_tutorial single_publisher_node_ns.launch 

SUMMARY
========

PARAMETERS
 * /publisher/topic_name: messageC
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES
  /
    publisher (launch_file_tutorial/launch_publisher)

auto-starting new master
process[master]: started with pid [4294]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to 5ba6bc48-f39a-11eb-a196-00155d5b842a
process[rosout-1]: started with pid [4309]
started core service [/rosout]
process[publisher-2]: started with pid [4318]
[ INFO] [1627912999.862952900]: /message 0
[ INFO] [1627913000.862954400]: /message 1
[ INFO] [1627913001.862946800]: /message 2
[ INFO] [1627913002.862955100]: /message 3
[ INFO] [1627913003.862946600]: /message 4
[ INFO] [1627913004.863272500]: /message 5
```

此处我们运行的是《动手学ROS（6）》中的一个简单的publisher，我们想要的是topic_name为messageC，但是程序没有获取到该参数，输出了默认值message。这不是我们想要的。日志中可以看到，节点启动后的parameters中有一个* /publisher/topic_name: messageC，它多了一个命名空间publisher，这个就是在launch文件中的node名字。

那我们应该怎么获取它呢？

代码如下：

```cpp
ros::init(argc, argv, "publisher");
    ros::NodeHandle n;
    ros::NodeHandle n1("~");
    std::string topicName;
    n.param<std::string>("topic_name", topicName, "message");
    n1.param<std::string>("topic_name", topicName, "default"); 
```

在定义NodeHandle时，加上一个 “~”，表示该handle的命名空间为私有空间，这样就可以正常获取参数了。

```bash
$ roslaunch launch_file_tutorial single_publisher_node_ns.launch 

SUMMARY
========

PARAMETERS
 * /publisher/topic_name: messageC
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES
  /
    publisher (launch_file_tutorial/launch_publisher)

auto-starting new master
process[master]: started with pid [4689]
ROS_MASTER_URI=http://localhost:11311

setting /run_id to b973bb72-f39b-11eb-9b1e-00155d5b842a
process[rosout-1]: started with pid [4704]
started core service [/rosout]
process[publisher-2]: started with pid [4713]
[ INFO] [1627913586.683735700]: /messageC 0
[ INFO] [1627913587.683896700]: /messageC 1
[ INFO] [1627913588.683910600]: /messageC 2
[ INFO] [1627913589.684092300]: /messageC 3
[ INFO] [1627913590.683876400]: /messageC 4
[ INFO] [1627913591.683896600]: /messageC 5
```

当然Nodehandle的参数也可以传入其它namespace。

#### 小结

私有空间也是一种相对空间，只是相对于自身节点。介绍这种作法主要是因为，这种方式可以**避免该节点的参数与其它节点发生名字冲突**，也不用自己明确设置namespace，例如使用group标签。

总之，这是一种常用的分组方式，希望读者以后见到时不会觉得陌生。

