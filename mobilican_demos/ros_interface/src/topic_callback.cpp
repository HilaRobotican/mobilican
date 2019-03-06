#include "topic_callback.h"


// void TopicCallback::subCallback(const type& msg)

void TopicCallback::subCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("I heard: [%s]", msg->data.c_str());
}

TopicCallback::CallbackFunc TopicCallback::get(int i)
{
  if (i==0){
    return &TopicCallback::subCallback; // TODO
  }
}
