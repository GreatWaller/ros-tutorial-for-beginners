# 动手学ROS（6）：命名空间之group与remap

命名空间（namespace）是编程语言中常用的一个名词，读者应该都不会陌生，它的主要作用就是避免命名冲突，也就是说，同一个Class可以存在不同的命名空间中。在ros中，命名空间的概念是相似的，同样也是为了区分名称，这个名称包括：

- 节点名称
- Topic名称
- Parameter名称

等等。ros系统的根命名空间是"/"。例如我们查看topic列表：

```bash
$ rostopic list
/A/message
/B/message
/rosout
/rosout_agg
```

最前面都是以"/"开头。同样，查看paramter列表：

```bash
$ rosparam list
/A/topic_name
/B/topic_name
/rosdistro
/rosversion
/run_id
```

可以看到同时有两个叫message的topic，但是因为在message前面添加了一层命名空间，使得避免了冲突，同时也实现的节点的复用。下面会详细说明实现方法。

#### 添加命名空间--group

我们使用上一节《动手学ROS（5）：roslaunch》中的示例：

```c++
std::string topicName;
n.param<std::string>("topic_name", topicName, "message");
ros::Publisher chatter_pub = n.advertise<std_msgs::String>(topicName, 100);
```

很简单的一个publisher，向默认名称为message的Topic发消息。如果以默认参数运行，发布的Topic名称为 /message，这是我们已经知道的事情。

接下来，是本节要重点介绍的group标签。在launch文件中，通过设置group的ns属性，可以使group内部包含的param或node，通通添加独立的命名空间。launch示例：

```xml
<launch>
    <group ns="A">
        <param name="topic_name" value="message"/>
        <node name="publisher" pkg="launch_file_tutorial" type="launch_publisher" output="screen"/>
    </group>
    <group ns="B">
        <param name="topic_name" value="message"/>
        <node name="publisher" pkg="launch_file_tutorial" type="launch_publisher" output="screen"/>
    </group>
</launch>
```

我们保持topic_name的值和node的name完全一致，分别添加不同的group, A和B，运行结果如下：

```bash
$ roslaunch launch_file_tutorial multi_publisher.launch 

SUMMARY
========

PARAMETERS
 * /A/topic_name: message
 * /B/topic_name: message
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES
  /A/
    publisher (launch_file_tutorial/launch_publisher)
  /B/
    publisher (launch_file_tutorial/launch_publisher)

process[A/publisher-2]: started with pid [26105]
[ INFO] [1621310742.998574500]: /A/message 0
process[B/publisher-3]: started with pid [26111]
[ INFO] [1621310743.083010200]: /B/message 0
[ INFO] [1621310743.498809400]: /A/message 1
[ INFO] [1621310743.583454900]: /B/message 1
[ INFO] [1621310743.998799000]: /A/message 2
[ INFO] [1621310744.083290500]: /B/message 2
```

适当删除了不必要的信息后，可以清楚地看到，parameters和nodes中添加了两个命名空间，使我们在不改动publisher的情况下，可同时运行两个节点，并往不同的Topic中发送消息。当时这不是唯一的作法（配置不同参数，更改不同的node name），却是比较好的作法，读者日后可在实践中多加体会。

#### Remap

由于Topic的应用无处不在，ros提供了remap命令来方便地修改topic的名称及命名空间。这里需要指出remap的两个主要用法：

- 更改topic名称
- 更改topic的命名空间

我们主要介绍第一种方式。在《动手学ROS（2）：Topic通信》中，我们写了一个简单的subscriber, 订阅了一个叫 chtter 的话题，现在就用remap来重新映射到B/message:

```xml
<launch>
    <node name="simple_subscriber" pkg="ros_topic" type="subscriber" output="screen">
        <remap from="chatter" to="B/message"/>
    </node>
</launch>
```

运行之后，启动刚才的multi_publisher.launch，同时发布两个Topic: A/message和B/message, 结果如下：

```bash
$ roslaunch launch_file_tutorial remap.launch
SUMMARY
========

PARAMETERS
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES
  /
    simple_subscriber (ros_topic/subscriber)

process[simple_subscriber-2]: started with pid [26198]
[ INFO] [1621318021.108896500]: I heard: [/B/message 1]
[ INFO] [1621318021.608953500]: I heard: [/B/message 2]
[ INFO] [1621318022.108827600]: I heard: [/B/message 3]
[ INFO] [1621318022.608819200]: I heard: [/B/message 4]
[ INFO] [1621318023.108907700]: I heard: [/B/message 5]
```

这里，subscriber只收到了名为B/message的Topic中的消息。

remap的另一种用途是更改Topic的命名空间，这里就不再举例演示了，详情请参考ros文档http://wiki.ros.org/ROS/Tutorials/UsingRqtconsoleRoslaunch，github地址在[ros_tutorials/turtlesim at noetic-devel · ros/ros_tutorials · GitHub](https://github.com/ros/ros_tutorials/tree/noetic-devel/turtlesim)

其中关键：

```xml
<launch>

  <group ns="turtlesim1">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <group ns="turtlesim2">
    <node pkg="turtlesim" name="sim" type="turtlesim_node"/>
  </group>

  <node pkg="turtlesim" name="mimic" type="mimic">
    <remap from="input" to="turtlesim1/turtle1"/>
    <remap from="output" to="turtlesim2/turtle1"/>
  </node>

</launch>
```

请看remap出现的地方，有input和output，这其实是Topic的初始命名空间，我们打开源码（mimic.cpp）：

```c++
Mimic::Mimic()
{
  ros::NodeHandle input_nh("input");
  ros::NodeHandle output_nh("output");
  twist_pub_ = output_nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);
  pose_sub_ = input_nh.subscribe<turtlesim::Pose>("pose", 1, &Mimic::poseCallback, this);
}
```

NodeHandle在初始化时传入了默认的命名空间，使得topic也添加了相应的命名空间，由于remap此时不能更改topic的名称，那publisher在实现时需要与subscriber保持一致，我们也可以找到相应的代码（turtle.cpp)：

```c++
  velocity_sub_ = nh_.subscribe("cmd_vel", 1, &Turtle::velocityCallback, this);
  pose_pub_ = nh_.advertise<Pose>("pose", 1);
```

#### 相对命名空间

到目前为止，我们无论在代码中还是launch文件中，使用的都是相对命名空间，**其特征是：不以"/"开头**。

如果获取参数的时候 `n.param<std::string>("/topic_name", topicName, "mess");`这样在刚才的示例中是查询不到的，因为启动multi_publisher.launch 时参数之前加上了命名空间，结果 如下：

```bash
$ roslaunch launch_file_tutorial multi_publisher.launch 
SUMMARY
========

PARAMETERS
 * /A/topic_name: message
 * /B/topic_name: message
 * /rosdistro: melodic
 * /rosversion: 1.14.10

NODES
  /A/
    publisher (launch_file_tutorial/launch_publisher)
  /B/
    publisher (launch_file_tutorial/launch_publisher)

process[A/publisher-2]: started with pid [14058]
[ INFO] [1621322206.010728400]: /A/mess 0
process[B/publisher-3]: started with pid [14064]
[ INFO] [1621322206.097387400]: /B/mess 0
[ INFO] [1621322206.511252900]: /A/mess 1
[ INFO] [1621322206.597566500]: /B/mess 1
[ INFO] [1621322207.010915800]: /A/mess 2
[ INFO] [1621322207.097526500]: /B/mess 2
[ INFO] [1621322207.511088900]: /A/mess 3
[ INFO] [1621322207.597536500]: /B/mess 3
```

由于没有查到"/topic_name"参数，而使用的默认值 mess。

**这样就清楚了，在代码里尽量使用相对命名空间，这样配合group或ns命令会比较灵活，但是有时必须要把名称限制到根空间下，那就需要使用"/"开关的名字，比如需要一个命名空间永不改变命名空间的的Topic，那在发布时就直接以"/"开头就可以了。**

#### 小结

1. 命名空间的作用是避免名称冲突，实现代码复用；
2. 在launch文件中可使用group标签，通过ns属性，添加节点或参数的命名空间；
3. remap可轻松更改Topic的名称和命名空间；
4. 相对命名空间的使用更加灵活。

