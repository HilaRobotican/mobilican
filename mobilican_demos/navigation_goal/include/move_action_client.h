#ifndef NAVIGATION_GOAL_MOVE_ACTION_CLIENT_H
#define NAVIGATION_GOAL_MOVE_ACTION_CLIENT_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <navigation_goal/MoveAction.h> // This is a header generated automatically from the FibonacciAction.msg file.


class MoveActionClient
{
public:

  /* Construct an action client. */
  MoveActionClient(ros::NodeHandle *nh);

  ~MoveActionClient(void);

  std::string getLocationName() const ;

  void sendGoal(std::string location_name);
  
  // Called once when the goal completes
  void doneCb(const actionlib::SimpleClientGoalState& state, const navigation_goal::MoveResultConstPtr& result);
  
  // Called once when the goal becomes active
  void activeCb();

  // Called every time feedback is received for the goal
  void feedbackCb(const navigation_goal::MoveFeedbackConstPtr& feedback);

private:
    ros::NodeHandle *nh_;

    actionlib::SimpleActionClient<navigation_goal::MoveAction> action_client_;

    std::string location_name_; // the name of the desired location

    void fetchParams();
};
#endif // NAVIGATION_GOAL_MOVE_ACTION_CLIENT_H
