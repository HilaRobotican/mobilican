#ifndef ROS_INTERFACE_ROS_INTERFACE_H
#define ROS_INTERFACE_ROS_INTERFACE_H

#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include <sstream>
#include "topic_callback.h"

/*
 * Represents an
 */
class RosInterface
{
public:
  // RosInterface(ros::NodeHandle *nh);
  virtual ~RosInterface() = 0;

  virtual void newPublisher(std::string topic_name) = 0;
  virtual void newSubscriber(std::string topic_name, TopicCallback& tc) = 0;
  virtual void publishToTopic(std::string topic_name) = 0;

private:
  ros::NodeHandle *nh_;

  std::vector<ros::Publisher> publishers_vec_;
  std::vector<ros::Subscriber> subscribers_vec_;

  std::map<std::string, int> publisher_to_index_; // each publisher is identified by its topic.
  // std::map<std::string, int> subscriber_to_index_;

};

inline RosInterface::~RosInterface() { } // prevent multiple definition

#endif // ROS_INTERFACE_ROS_INTERFACE_H
