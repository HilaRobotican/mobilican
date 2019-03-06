#include "vision.h"




Vision::Vision(ros::NodeHandle *nh)
{
  ros_imp_ = new Ros1Implementation(nh); // TODO is ok? another place with ros1.
  // tc_ = new TopicCallback();
}

Vision::~Vision()
{
  delete ros_imp_;
  // delete tc_;
}


// void Vision::subCallback(const std_msgs::String::ConstPtr& msg) //TODO generelized.
//   {
//     ROS_INFO("I heard: [%s]", msg->data.c_str());
//   }

void Vision::newPublisher(std::string topic_name)
{
  ros_imp_->newPublisher(topic_name);
}

void Vision::newSubscriber(std::string topic_name)
{
  // ros_imp_->newSubscriber(topic_name, &Vision::func);
  ros_imp_->newSubscriber(topic_name, tc_);
}


// // typedef void (* CallbackFunc)(const std_msgs::String::ConstPtr& msg);
// // typedef void (* CallbackFunc)(const geometry_msgs::PoseStampedConstPtr& msg);
// CallbackFunc Vision::f(int i)
// {
//   if (i == 0)
//   {
//     return &Vision::subCallback;
//   }
// }

void Vision::publishToTopic(std::string topic_name)
{
  ros_imp_->publishToTopic(topic_name);
}
//
// void Vision::subCallback(const std_msgs::String::ConstPtr& msg)
// {
//     ROS_INFO("I heard: [%s]", msg->data.c_str());
// }
