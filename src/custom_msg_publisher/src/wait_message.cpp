#include "ros/ros.h"
#include "public_pkg/CustomMsg.h"
#include <boost/shared_ptr.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wait_message");
    ros::NodeHandle n;
    ros::Rate loop_rate(1);

    while (ros::ok())
    {
        boost::shared_ptr<public_pkg::CustomMsg const> msg;
        msg = ros::topic::waitForMessage<public_pkg::CustomMsg>("custom_topic", ros::Duration(5));

        if (msg)
        {
            ROS_INFO("%s %d", msg->name.c_str(), msg->age);
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}