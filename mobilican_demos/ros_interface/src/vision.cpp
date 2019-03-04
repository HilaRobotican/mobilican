#include "vision.h"




Vision::Vision(ros::NodeHandle *nh)
{
  ros_imp_ = new Ros1Implementation(nh); // TODO is ok? another place with ros1.
}

Vision::~Vision()
{
  delete ros_imp_;
}


void Vision::subCallback(const std_msgs::String::ConstPtr& msg) //TODO generelized.
  {
    ROS_INFO("I heard: [%s]", msg->data.c_str());
  }

void Vision::newPublisher(std::string topic_name)
{
  ros_imp_->newPublisher(topic_name);
}

void Vision::newSubscriber(std::string topic_name)
{
  ros_imp_->newSubscriber(topic_name);
}

void Vision::publishToTopic(std::string topic_name)
{
  ros_imp_->publishToTopic(topic_name);
}
