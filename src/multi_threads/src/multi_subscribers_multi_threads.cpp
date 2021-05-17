#include <thread>
#include <ros/callback_queue.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
void CallbackA(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
void CallbackB(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub_b = n.subscribe("MessageB", 1, CallbackB);
  
  ros::NodeHandle n_a;
  ros::CallbackQueue callback_queue_a;
  n_a.setCallbackQueue(&callback_queue_a);
  ros::Subscriber sub_a = n_a.subscribe("MessageA", 1, CallbackA);
  std::thread spinner_thread_a([&callback_queue_a]() {
    ros::SingleThreadedSpinner spinner_a;
    spinner_a.spin(&callback_queue_a);
  });
  ros::spin();
  spinner_thread_a.join();
  return 0;
}