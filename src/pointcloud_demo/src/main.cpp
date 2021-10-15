#include "pointcloud_pub_sub.hpp"

int main(int argc, char **argv)
{
    ros::init(argc,argv,"pointcloud_demo");
    ros::NodeHandle nh;
    ros::Rate rate(1);
    pointcloud_pub_sub pointcloud_demo(&nh);

    for (size_t i = 0; i < 10; i++)
    {
        pointcloud_demo.publish();
        ros::spinOnce();
        rate.sleep();
    }
    
    // ros::spin();
    return 0;
}
