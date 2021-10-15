#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
// #include <pcl_ros/transforms.h>

class pointcloud_pub_sub
{
private:
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::string pointcloud_topic = "cloudpoint_topic";

    void initializePublishers();
    void initializeSubscribers();
    void callback(const sensor_msgs::PointCloud2ConstPtr &cloud);

public:
    pointcloud_pub_sub(ros::NodeHandle *node);
    ~pointcloud_pub_sub();
    void publish();
};

pointcloud_pub_sub::pointcloud_pub_sub(ros::NodeHandle *node) : nh(*node)
{
    initializePublishers();
    initializeSubscribers();
}

pointcloud_pub_sub::~pointcloud_pub_sub()
{
}

void pointcloud_pub_sub::initializePublishers()
{
    pub = nh.advertise<sensor_msgs::PointCloud2>(pointcloud_topic, 10);
}

void pointcloud_pub_sub::initializeSubscribers()
{
    sub = nh.subscribe(pointcloud_topic, 10, &pointcloud_pub_sub::callback, this);
}

void pointcloud_pub_sub::callback(const sensor_msgs::PointCloud2ConstPtr &cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*cloud, *temp_cloud);
    // or
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*cloud, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

    //do stuff with temp_cloud using pcl library
    ROS_INFO("received %ld points", temp_cloud->points.size());

}

void pointcloud_pub_sub::publish()
{
    sensor_msgs::PointCloud2 msg;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    for (size_t i = 0; i < 3; i++)
    {
        pcl::PointXYZ p(1.0 * i, 2.0 * i, 3.0 * i);
        cloud->push_back(p);
    }

    pcl::toROSMsg(*cloud, msg);
    // or
    pcl::PCLPointCloud2 pcl_pc2;
    pcl::toPCLPointCloud2(*cloud, pcl_pc2);
    pcl_conversions::fromPCL(pcl_pc2, msg);

    // publish
    pub.publish(msg);
    ROS_INFO("published.");
}