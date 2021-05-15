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