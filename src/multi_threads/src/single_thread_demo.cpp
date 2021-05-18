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