#ifndef ROS_INTERFACE_TOPIC_CALLBACK_H
#define ROS_INTERFACE_TOPIC_CALLBACK_H

#include <ros/ros.h>
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PoseStamped.h"
#include <string>
#include "std_msgs/String.h"
#include "boost/variant.hpp"

/*
 * Represents an
 */
class TopicCallback
{
public:
    TopicCallback(){}
    ~TopicCallback(){}
    // virtual ~TopicCallback() = 0;

    // callback funstions
    typedef boost::variant<geometry_msgs::PoseStamped::ConstPtr, std_msgs::String::ConstPtr> type; // TODO - ADD msgs


    // void subCallback(const type& msg);  // virtual with empty default implementation
    void subCallback(const std_msgs::String::ConstPtr& msg);  // virtual with empty default implementation


    // typedef void (* CallbackFunc)(const type& msg);
    //     typedef void (* CallbackFunc)(const std_msgs::String::ConstPtr& msg);
    typedef void (TopicCallback::*CallbackFunc)(const std_msgs::String::ConstPtr& msg);
    CallbackFunc get(int i);


  // typename typename
  // template<typename ROSMessageType>
  // void subCallback(const typename ROSMessageType::ConstPtr& msg);  // virtual with empty default implementation
  //
  // // virtual void subCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {}  // virtual with empty default implementation
  //
  // template<typename ROSMessageType>
  // typename  using CallbackFunc = void (* )(const typename ROSMessageType::ConstPtr& msg);
  //
  // // struct TypeHelper{
  // //   typedef void (* CallbackFunc)(const typename ROSMessageType::ConstPtr& msg);
  // // };
  //
  // template<typename ROSMessageType>
  // CallbackFunc<geometry_msgs::PoseStamped> get(int i);

};
// inline TopicCallback::~TopicCallback() { } // prevent multiple definition

#endif // ROS_INTERFACE_VISION_H
