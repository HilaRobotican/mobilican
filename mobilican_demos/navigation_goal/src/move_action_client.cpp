#include "move_action_client.h"

// usage: rosrun my_action move_callback_client location_name:=____


/* Construct an action client that listen to "move_server" node. */
MoveActionClient::MoveActionClient(ros::NodeHandle *nh) : nh_(nh), action_client_("move_action_server", true)
{
  fetchParams();
  ROS_INFO("Waiting for action server to start.");
  action_client_.waitForServer();
  ROS_INFO("Action server started, sending goal.");
}

MoveActionClient::~MoveActionClient(void){};

void MoveActionClient::fetchParams()
{
  nh_->getParam("location_param", location_name_);
  ROS_INFO_STREAM("got param: " << location_name_);
}

std::string MoveActionClient::getLocationName() const
{
  return location_name_;
}


void MoveActionClient::sendGoal(std::string location_name)
{
  navigation_goal::MoveGoal goal;
  goal.location_name = location_name;

  // Need boost::bind to pass in the 'this' pointer.
  // _1, _2 stands for the function arguments.
  action_client_.sendGoal(goal,
                          boost::bind(&MoveActionClient::doneCb, this, _1, _2),
                          boost::bind(&MoveActionClient::activeCb, this),
                          boost::bind(&MoveActionClient::feedbackCb, this, _1));
  //                          actionlib::SimpleActionClient<my_action::MoveActionServer>::SimpleActiveCallback(),
//                          actionlib::SimpleActionClient<my_action::MoveActionServer>::SimpleFeedbackCallback());
}


// Called once when the goal completes
void MoveActionClient::doneCb(const actionlib::SimpleClientGoalState& state, 
            const navigation_goal::MoveResultConstPtr& result)
{
  ROS_INFO_STREAM("Finished in state " << state.toString().c_str() << " " << result->res);
  ros::shutdown();
}

// Called once when the goal becomes active
void MoveActionClient::activeCb()
{
  ROS_INFO("Goal just went active");
}

// Called every time feedback is received for the goal
void MoveActionClient::feedbackCb(const navigation_goal::MoveFeedbackConstPtr& feedback)
{
  ROS_INFO_STREAM("Got feedback: " << feedback->name);
}