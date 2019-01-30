#include <ros/ros.h>
#include "move_action_client.h"

/**
 * The main function. Create an object of the class and send the goal - location name
 * according to the param "location_param". 
 */
int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_client_node");
  
  // Create node handler for getting the name of the desired location
  ros::NodeHandle nh("~");

  // Create move action client object that will fetch the param from the user
  // and send the goal to the action server.
  MoveActionClient client(&nh);
  client.sendGoal(client.getLocationName());

  ros::spin();
  return 0;
}