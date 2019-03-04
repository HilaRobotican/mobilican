#include <ros/ros.h>
#include "vision.h"

// usage: roslaunch my_action my_action.launch location_name:=tal


/**
 * The main function. Create an object of the class and load the yaml file.
 * Then waits for the client's goal.
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_node");
  ros::NodeHandle nh;

  Vision v(&nh);

  std::string topic_name = "chatter";
  v.newPublisher(topic_name);
  v.newSubscriber(topic_name);
  v.publishToTopic(topic_name);

  ros::spin();

  return 0;
}
