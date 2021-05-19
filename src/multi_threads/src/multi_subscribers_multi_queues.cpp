#include <thread>
#include <ros/callback_queue.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
void CallbackA(const std_msgs::String::ConstPtr& msg) {
  std::this_thread::sleep_for(std::chrono::seconds(2));
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
void CallbackB(const std_msgs::String::ConstPtr& msg) {
  ROS_INFO(" I heard: [%s]", msg->data.c_str());
}
int main(int argc, char **argv) {
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;
  ros::Subscriber sub_a = n.subscribe("A/message", 1, CallbackA);
  
  ros::NodeHandle n_b;
  ros::CallbackQueue callback_queue_b;
  n_b.setCallbackQueue(&callback_queue_b);
  ros::Subscriber sub_b = n_b.subscribe("B/message", 1, CallbackB);
  std::thread spinner_thread_b([&callback_queue_b]() {
    ros::SingleThreadedSpinner spinner_b;
    spinner_b.spin(&callback_queue_b);
  });
  ros::spin();
  spinner_thread_b.join();
  return 0;
}