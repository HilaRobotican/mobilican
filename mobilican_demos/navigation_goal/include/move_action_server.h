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
#include "navigation_goal/ImageSnapshot.h" // This is a header generated automatically from the FibonacciAction.msg file.
#include "nav_msgs/Odometry.h"
#include "point.h"


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

#define LOCATIONS_CONFIG_PARAM "locations"

/* 
 * Represents an action server that receives a name of a location from the action client,
 * and searches for its x, y, and Y coordinates (after loading this information from a Yaml file). 
 * If the location exists, sends it as a goal to move_base node (which is an action server).
 * Therefore it is also a client of move_base.
 * Moreover, it is also a client of image_snapshot_node, meaning that each time that the robot reaches
 * to its goal, it takes snapshot of the cameras.
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

  ros::ServiceClient image_snapshot_client_;

  // subscriber to odom topic.
  ros::Subscriber odom_subscriber_;

  std::string action_name_;


  double cur_linear_vel_;
  double cur_angular_vel_;

  navigation_goal::MoveFeedback feedback_;
  navigation_goal::MoveResult result_;

  // variables for the yaml file parsing
  XmlRpc::XmlRpcValue locations_; // the name of the yaml file
//  std::map<std::string, point> locations_map_;
  std::map<std::string, int> locations_map_; // map a location name to its index in the vector.
  std::vector<point> points_vec_;
  point cur_point_;  // represents the desired location
  int index_counter_; // counts the elements in the yaml file.
  move_base_msgs::MoveBaseGoal goal_; // goal to move base

  void initActionServer();

  /* Initialize the move_base client. */
  void initMoveBaseClient();

  // Called whenever a new message arrives from the odom topic.
  void initOdomSubscriber();

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

  /* Validate that the robot stopped.*/
  bool validateStop();

  /* Called when the image_snapshot client send a request to image_snapshot_node. */
  void callImageSnapshot();

  void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

  void executeAllGoals(const navigation_goal::MoveGoalConstPtr &goal);

};
#endif // NAVIGATION_GOAL_MOVE_ACTION_SERVER_H
