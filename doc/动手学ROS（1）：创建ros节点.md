# 动手学ROS（1）：创建ros节点

ROS (Robot Operating System, 机器人操作系统) 提供一系列程序库和工具以帮助软件开发者创建机器人应用软件。它提供了硬件抽象、设备驱动、库函数、可视化、消息传递和软件包管理等诸多功能。

#### 1  安装 ros 

安装步骤请参考 [安装教程](http://wiki.ros.org/melodic/Installation/Ubuntu)

该页面是ROS Melodic版本。本系列教程提供的代码均为在Ubuntu18.04上实测通过。

#### 2 创建ros node

##### 2.1 创建catkin workspace:

```bash
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws/
$ catkin_make
```

初次运行catkin_make命令会在src目录下生成CMakeLists.txt。该文件不需要改动。

##### 2.2 创建package

```bash
$ cd ~/catkin_ws/src
$ catkin_create_pkg ros_tutorial std_msgs roscpp
```

需要注意的是，创建pkg时需要进入src目录。ros_tutorial为pkg的名称，std_msgs 及roscpp为pkg的所依赖的pkg，我们暂时使用c++来进行代码示例，所以依赖了roscpp。顺便提一下，std_msgs为ros提供的标准通信格式，后续讲到通信方式的时候再来细说。

我们首先来认识下pkg默认的目录结构

```bash
└── ros_tutorial
    ├── CMakeLists.txt
    ├── include
    │   └── ros_tutorial
    ├── package.xml
    └── src
```

目前我们需要注意到，新生成的pkg里也有一个CMakeLists.txt，这个文件是需要修改的。熟悉c++的朋友应该可以看出来，这就是一个普通的cmake文件，负责工程的编译。

我们写的cpp文件就写在src目录下，头文件放在include下，package.xml用来维护依赖pkg。

##### 2.3 编写ros node

ros node 可以简单理解为ros通信的基本单位。ros工程运行时有一个master 节点和若干个 client节点，各client之间通过master进行通信。

下面就简单写一个ros node. 在ros_tutorial/src下新建ros_tutorial_node.cpp文件：

```c++
#include <ros/ros.h>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_tutorial_node");
    ros::NodeHandle n;
    ros::Rate rate(1);

    while (n.ok())
    {
        ROS_INFO("spin once");
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
```

其实这里可以随便写一个cpp文件，这里主要是为了引入ros::nodeHandle的概念。

- ros::init

  初始化

- ros::NodeHandle

  创建ros节点

- ros::Rate

  节点刷新频率

- ros::spinOnce 或ros::spin

  执行刷新

以上是启动节点的必要操作，细节后续会逐步介绍。

##### 2.4 修改CMakeLists

主要修改两个地方。反注释就可以了。

- add_executable：编译目标
- target_link_libraries：定义链接库

```cmak
## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node src/ros_tutorial_node.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
```

##### 2.5 编译并执行

###### 2.5.1 build

build需要在workspace下进行，故先返回上级目录，再执行catkin_make

```bash
$ cd ..
$ catkin_make
```

build成功后，可留意一下workspace下的build和devel文件夹，编译的结果在devel中，build主要记录了cmake相关编译文件。

###### 2.5.2 run

首先需要启动roscore,也就是之前提到过的ros master 节点。打开一个新的终端：

```bash
$ roscore
```

然后在之前的终端中输入：

```bash
$ source devel/setup.bash
$ rosrun ros_tutorial ros_tutorial_node
```

初学ros的时候经常会忘记source，养成习惯就好了。

最后，输出结果：

```bash
$ rosrun ros_tutorial ros_tutorial_node
[ INFO] [1620964595.224187100]: spin once
[ INFO] [1620964596.224587500]: spin once
[ INFO] [1620964597.224583200]: spin once
[ INFO] [1620964598.224580900]: spin once
[ INFO] [1620964599.224578200]: spin once
```

代码的意思是 每秒写一条日志。

#### 3 结语

本文快速地把学习ros需要的最基本操作串了起来，接下来会介绍ros的通信方式。

本系列教程是笔者过去半年的ros开发经验总结，记录成文供读者参考，欢迎留言交流。