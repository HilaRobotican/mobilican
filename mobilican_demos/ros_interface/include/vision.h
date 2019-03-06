#ifndef ROS_INTERFACE_VISION_H
#define ROS_INTERFACE_VISION_H

#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "ros1_implementation.h"
#include "ros_interface.h"
#include "topic_callback.h"
/*
 * Represents an
 */
class Vision
{
public:
  Vision(ros::NodeHandle *nh);
  ~Vision(void);
  void newPublisher(std::string topic_name);
  void newSubscriber(std::string topic_name);
  void publishToTopic(std::string topic_name);

  // callback funstions
  // void subCallback(const std_msgs::String::ConstPtr& msg); //TODO here??

  TopicCallback tc_;

  // virtual void subCallback(const std_msgs::String::ConstPtr& msg);  // virtual with empty default implementation

  // virtual void subCallback(const geometry_msgs::PoseStampedConstPtr& msg) {}  // virtual with empty default implementation

  // typedef void (* CallbackFunc)(const std_msgs::String::ConstPtr& msg);
  // typedef void (* CallbackFunc)(const geometry_msgs::PoseStampedConstPtr& msg);
  // CallbackFunc f(int i)

private:
  RosInterface* ros_imp_;



};
#endif // ROS_INTERFACE_VISION_H
