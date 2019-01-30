#include <ros/ros.h>
#include "move_action_server.h"

// usage: roslaunch my_action my_action.launch location_name:=tal


/**
 * The main function. Create an object of the class and load the yaml file. 
 * Then waits for the client's goal. 
 */
int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_server_node");
  ros::NodeHandle nh;

  MoveActionServer move_server(&nh, "move_action_server"); // move action server and move_base client.

  // ready to receive goal from client and send it to move_base.
  ROS_INFO("Server started");
  ros::spin();

  return 0;
}
