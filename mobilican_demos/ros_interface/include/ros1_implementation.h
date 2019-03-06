#ifndef ROS_INTERFACE_ROS1_IMPLEMENTATION_H
#define ROS_INTERFACE_ROS1_IMPLEMENTATION_H

#include <ros/ros.h>
// #include <string>
#include "ros_interface.h"

// #include <sstream>

/*
 * Represents an
 */
class Ros1Implementation : public RosInterface
{
public:
  Ros1Implementation(ros::NodeHandle *nh);
  virtual ~Ros1Implementation(void);

  virtual void newPublisher(std::string topic_name);

  // template<typename ROSMessageType>
  virtual void newSubscriber(std::string topic_name, TopicCallback& tc);

  virtual void publishToTopic(std::string topic_name);

  // template<typename ROSMessageType>
  // void subCallback(const typename ROSMessageType::ConstPtr& msg); //TODO should not be here!
  // // void subCallback(const std_msgs::String::ConstPtr& msg);  //TODO should not be here!

private:
  ros::NodeHandle *nh_;

  std::vector<ros::Publisher> publishers_vec_;
  std::vector<ros::Subscriber> subscribers_vec_;

  std::map<std::string, int> publisher_to_index_; // each publisher is identified by its topic.
  // std::map<std::string, int> subscriber_to_index_;

};
#endif // ROS_INTERFACE_ROS1_IMPLEMENTATION_H
