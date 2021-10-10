# 动手学ROS（11）：图像传输

计算机视觉（CV）是机器人系统中必不可少的一环，我们先从最简单的图像传输讲起。

对于双目相机标定、手眼标定、深度相机（或激光雷达）与RGB相机的标定等基础但重要的方法，会陆续涉及到，物体检测及定位等如果有时间也会整理出来与读者交流（好像给自己挖了个大坑）。

本系列教程的一贯原则，是用最简单的示例来介绍最基础但同时也是最重要的知识点，增加可操作性，而后举一反三，融会贯通。

因此，本节就从读取一张jpg图片，并发送至Topic讲起。

#### Publisher

我们之前讲过Topic的使用方法，所以我们已经知道要怎么实现了：

```python
#!/usr/bin/env python2
import rospy
from sensor_msgs.msg import Image
import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np


if __name__ == '__main__':

    pub = rospy.Publisher('camera/image', Image, queue_size=10)
    rospy.init_node('image_topic_publisher_node', anonymous=True)
    rate = rospy.Rate(10)  # 10hz

    bridge = CvBridge()
    while not rospy.is_shutdown():
        try:
            # read a frame
            img=cv2.imread("image.jpg")
            msg = bridge.cv2_to_imgmsg(img, encoding="bgr8")

            pub.publish(msg)
            rate.sleep()
        except rospy.ROSInterruptException:
            pass
```

我们还是第一次用python来示例。ROS本身支持c++与python，因此有些用python实现比较方便的情况，不防就用python。需要强调的是在ROS Melodic中，请使用python2.7来运行，其它版本可能没有这个要求，具体没了解过。运行方法可以使用传统的python命令（python2 xx.py)也可以写launch文件。

不过，ROS提供了一个专门传输图像的package：image_transport。详情请见http://wiki.ros.org/image_transport/Tutorials。

那我们重写这个publisher：

```c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "image_publisher");
  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Publisher pub = it.advertise("camera/image", 1);
  cv::Mat image = cv::imread(argv[1], CV_LOAD_IMAGE_COLOR);
  cv::waitKey(30);
  sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image).toImageMsg();

  ros::Rate loop_rate(2);
  while (nh.ok()) {
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
}
```

这里用的OpenCV 3.4版本，同时ROS提供了一个叫cv_bridge的package，这里面提供了方便的函数来进行OpenCV的Mat与ROS的Msg的格式转换，比如这里的cv_bridge::CvImage就可以直接把cv::Mat转换为sensor_msgs::Image。

运行：

```
rosrun image_demo image_publisher_node image.jpg
```

查看一下生成的topic:

```bash
$ rostopic list
/camera/image
/camera/image/compressed
/camera/image/compressed/parameter_descriptions
/camera/image/compressed/parameter_updates
/camera/image/compressedDepth
/camera/image/compressedDepth/parameter_descriptions
/camera/image/compressedDepth/parameter_updates
/camera/image/theora
/camera/image/theora/parameter_descriptions
/camera/image/theora/parameter_updates
```

此处我们仅关注/camera/image。首先可以查看一下本例的图片：

```bash
$ rqt_image_view
```

![image-20210812094603853](https://raw.githubusercontent.com/GreatWaller/ros-tutorial-for-beginners/main/doc/images/image-20210812094603853.png)

#### Subscriber

相应地，我们来订阅"camera/image"：

```c++
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "image_listener");
  ros::NodeHandle nh;
  cv::namedWindow("view");

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("camera/image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow("view");
}
```

就不做过多的解释了。

#### CMakeLists.txt

为了让读者能快速运行复现，由于是第一次引入其它的依赖库，我们把cmake文件也放在这里：

```cmake
cmake_minimum_required(VERSION 3.0.2)
project(image_demo)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  cv_bridge
  image_transport
)

## System dependencies are found with CMake's conventions
# find_package(Boost REQUIRED COMPONENTS system)
find_package(OpenCV 3 REQUIRED)
MESSAGE(STATUS "OpenCV version: "${OpenCV_VERSION})
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES image_demo
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)
add_executable(image_publisher_node src/image_publisher.cpp)
target_link_libraries(image_publisher_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )

add_executable(image_subscriber_node src/image_subscriber.cpp)
target_link_libraries(image_subscriber_node 
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  )
```

#### 小结

要说image_transport的优势，文档中是有指出的。简单来说，它提供了可自由设置的图像压缩能力，以调控网络传输性能。我们如果用不到这个功能，就姑且把它当成一个普通的Topic来用就好。

另外，如果读者想用实际的相机来测试，比如usb相机，可参考http://wiki.ros.org/image_transport/Tutorials/PublishingImages 中Adding video stream from a webcam章节。此处就不赘述。

