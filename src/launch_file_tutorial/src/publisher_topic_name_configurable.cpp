#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "publisher");
    ros::NodeHandle n;

    std::string topicName;
    n.param<std::string>("topic_name", topicName, "message");
    // n.param<std::string>("/topic_name", topicName, "mess");
    // n.param<std::string>("/topic_name", topicName, "/message");

    ros::Publisher chatter_pub = n.advertise<std_msgs::String>(topicName, 100);

    ros::Rate loop_rate(1);

    int count = 0;
    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        std::string ns = n.getNamespace().compare("/") == 0 ? "" : n.getNamespace();
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