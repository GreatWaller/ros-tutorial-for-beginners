# 动手学ROS——自定义msg

工程中由于自定义的msg或srv比较多，通常会单独把自定义的内容放入一个pkg。本节我们新建一个叫public_pkg的pkg。

```bash
$ catkin_create_pkg public_pkg std_msgs roscpp
```

#### 1 自定义msg文件

##### 新建目录msg, 并新增我们自定义的CustomMsg.msg

```
string name
uint8 age
```

##### 修改CMakeLists.txt

确保以下几项与本文相同

```cmake
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)
add_message_files(
  FILES
  CustomMsg.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES public_pkg
 CATKIN_DEPENDS roscpp std_msgs message_runtime
#  DEPENDS system_lib
)
```

##### 修改package.xml

新增

```xml
  <build_depend>message_generation</build_depend>
  <exec_depend>message_runtime</exec_depend>
```

编译完成后，devel目录下会生成include/public_pkg/CustomMsg.h, 此文件就是我们将要使用的真正的类型文件。

#### 2 自定义msg的使用

自然我们是在另一个pkg中使用刚刚定义的CustomMsg, 为此，新增一个pkg是必要的，取名custom_msg_publisher, 从名字可以看出，我们将发布一个CustomMsg类型的Topic。

##### 新建pkg

```bash
$ catkin_create_pkg custom_msg_publisher std_msgs roscpp public_pkg
```

新建pkg时直接写进去依赖public_pkg会省去很多配置的麻烦，当然前提是事先已经已知pkg的名字。如果以后msg所在pkg名字更换或换了位置，就需要自行配置了，因此这里还是把需要注意的地方提出来。

- package.xml

新增如下

```
  <build_depend>public_pkg</build_depend>
  <build_export_depend>public_pkg</build_export_depend>
  <exec_depend>public_pkg</exec_depend>
```

- CMakeLists.txt

```
find_package(catkin REQUIRED COMPONENTS
  public_pkg
  roscpp
  std_msgs
)

```



##### 新建publisher

在custom_msg_publisher/src下新建custom_msg_publisher_node.cpp

```c++
#include "ros/ros.h"
#include "public_pkg/CustomMsg.h"

int main(int argc, char **argv)
{
    ros::init(argc, argv, "custom_topic");
    ros::NodeHandle n;

    ros::Publisher custom_pub = n.advertise<public_pkg::CustomMsg>("custom_topic", 100);

    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        public_pkg::CustomMsg msg;

        msg.name = "hello world ";
        msg.age = count;
        ROS_INFO("%s %d", msg.name.c_str(), msg.age);

        custom_pub.publish(msg);

        ros::spinOnce();

        loop_rate.sleep();
        ++count;
    }

    return 0;
}
```

熟悉的栗子，再拿出来炒炒。

请留意

- #include "public_pkg/CustomMsg.h"

- Publisher类型也相应更换为public_pkg::CustomMsg

- 使用msg里的属性

  msg.name = "hello world ";

  msg.age = count;

###### 修改CMakeLists.txt

其实就是反注释以下3句，我在文件命名的时候就以默认的规范，就不需要再对命名进行修改了。

```cmake
add_executable(${PROJECT_NAME}_node src/custom_msg_publisher_node.cpp)
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
)
```

其实${PROJECT_NAME}_EXPORTED_TARGETS}是不需要的，该pkg不依赖于自己生成的msg头文件，保留${catkin_EXPORTED_TARGETS}就可以了。但是留着也无妨，操作起来也省事。

#### 3 测试

编译后执行

```bash
$ rosrun custom_msg_publisher custom_msg_publisher_node
[ INFO] [1620984110.914541000]: hello world  0
[ INFO] [1620984111.915023800]: hello world  1
[ INFO] [1620984112.914891700]: hello world  2
[ INFO] [1620984113.914629300]: hello world  3
[ INFO] [1620984114.914943900]: hello world  4
```

收工！

最后，顺便介绍一个常用的ros命令 rostopic, 它可以查看、发布Topic。

- rostopic list 查看全部Topic

```bash
$ rostopic list
/custom_topic
/rosout
/rosout_agg
```

- rostopic echo 查看某一Topic的内容

```bash
$ rostopic echo /custom_topic 
name: "hello world "
age: 1
---
name: "hello world "
age: 2
```

`报错的肯定是忘了 source devel/setup.bash`

#### 4 小结

到这里，topic的最基础用法已经介绍完了，下一节讨论一下取topic中msg的不同方式。