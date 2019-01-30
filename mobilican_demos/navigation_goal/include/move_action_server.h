#ifndef NAVIGATION_GOAL_MOVE_ACTION_SERVER_H
#define NAVIGATION_GOAL_MOVE_ACTION_SERVER_H

#include <ros/ros.h>
#include <string> 
#include <actionlib/server/simple_action_server.h>
#include <actionlib/client/simple_action_client.h>
#include <XmlRpcValue.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include "navigation_goal/MoveAction.h" // This is a header generated automatically from the FibonacciAction.msg file.
#include "point.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define LOCATIONS_CONFIG_PARAM "locations"

/* 
 * Represents an action server that receives a name of a location from the action client,
 * and searches for its x, y, and Y coordinates (after loading this information from a Yaml file). 
 * If the location exists, sends it as a goal to move_base node (which is an action server). 
 */
class MoveActionServer
{
public:

  /* Construct an action server. */
  MoveActionServer(ros::NodeHandle *nh, std::string name);

  ~MoveActionServer(void);

private:
  
  // The node handle is constructed and passed into the action server during construction of the action.
  // The feedback and result messages are created for publishing in the action.
  ros::NodeHandle *nh_;
  actionlib::SimpleActionServer<navigation_goal::MoveAction> * action_server_;
  MoveBaseClient * move_base_client_;
  std::string action_name_;

  navigation_goal::MoveFeedback feedback_;
  navigation_goal::MoveResult result_;

  // variables for the yaml file parsing
  XmlRpc::XmlRpcValue locations_; // the name of the yaml file
  std::map<std::string, point> locations_map_;
  point p_;  // represents the desired location

  move_base_msgs::MoveBaseGoal goal_; // goal to move base

  void initActionServer();

  /* Initialize the move_base client. */
  void initMoveBaseClient();

  // Check if the yaml file is valid. If not, exit.
  bool validateYamlType(XmlRpc::XmlRpcValue::Type actual_type, XmlRpc::XmlRpcValue::Type wanted_type);
  
  void fetchParams();
  
  /* Parse yaml file and load the locations */
  void loadLocations();

  /*
   * The callback function. Called when the client send a goal. 
   */
  void executeCB(const navigation_goal::MoveGoalConstPtr &goal);

  void createGoalToMoveBase();

  /* Send a goal to the robot to move to a specific location */
  void publishGoal();

};
#endif // NAVIGATION_GOAL_MOVE_ACTION_SERVER_H
