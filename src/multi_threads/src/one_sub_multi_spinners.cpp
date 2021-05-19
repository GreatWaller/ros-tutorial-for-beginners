#include <thread>
#include "ros/ros.h"
#include "std_msgs/String.h"

void ChatterCallback(const std_msgs::String::ConstPtr& msg) {
  std::this_thread::sleep_for(std::chrono::seconds(2));
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::SubscribeOptions ops;
  ops.init<std_msgs::String>("A/message", 1, ChatterCallback);
  ops.allow_concurrent_callbacks = true;
  ros::Subscriber sub1 = n.subscribe(ops);
  ros::MultiThreadedSpinner spinner(2);
  spinner.spin();
  return 0;
}