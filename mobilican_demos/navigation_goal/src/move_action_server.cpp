#include "move_action_server.h"

// usage: roslaunch navigation_goal navigation_goal.launch location_name:=___
//usage:
// roslaunch mobilican komodo_2.launch gazebo:=true lidar:=true move_base:=true amcl:=true world_name:="/home/hila/catkin_ws/src/mobilican/mobilican_gazebo/worlds/rooms.world" have_map:=true map:="/home/hila/catkin_ws/src/mobilican/mobilican_navigation/maps/rooms.yaml"
//rviz


void MoveActionServer::initActionServer()
{
    action_server_ = new actionlib::SimpleActionServer<navigation_goal::MoveAction>(*nh_, action_name_,
                        boost::bind(&MoveActionServer::executeCB, this, _1), false);
    action_server_->start();

    // load the locations yaml file
    fetchParams();
    loadLocations();
}

/* Initialize the move_base client.
 * This client sends a goal, i.e. the coordinates of a location (position + orientation),
 * to the "move_base" node, which plays here as a server.
 */
void MoveActionServer::initMoveBaseClient()
{
    // create an action client that send to "move_base" server
    // true causes the client to spin its own thread
    move_base_client_ = new MoveBaseClient("move_base", true);

    //wait for the action server ("move_base") to come up
    while(!move_base_client_->waitForServer())
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }
}

/* Construct an action server. */
MoveActionServer::MoveActionServer(ros::NodeHandle *nh, std::string name) : //action_server_(nh_, name, boost::bind(&MoveActionServer::executeCB, this, _1), false),
                                       nh_(nh), action_name_(name), index_counter_(0)
{
    initActionServer();
    initMoveBaseClient();
}

MoveActionServer::~MoveActionServer(){
  delete action_server_;
  delete move_base_client_;
}

// Check if the yaml file is valid. If not, exit.
bool MoveActionServer::validateYamlType(XmlRpc::XmlRpcValue::Type actual_type, XmlRpc::XmlRpcValue::Type wanted_type)
{
  return actual_type == wanted_type;
}

void MoveActionServer::fetchParams()
{
  // TODO - TO CHECK THE PRINTING OF ERRORS IN THIS FUNCTION.
  /* LOCATIONS_CONFIG_PARAM */
  if (!nh_->hasParam(LOCATIONS_CONFIG_PARAM))
  {
    ROS_ERROR("[MoveActionServer]: %s param is missing on param server. "
              "Make sure that your launch includes this param file. "
              "shutting down...", LOCATIONS_CONFIG_PARAM);
    ros::shutdown();
    exit(EXIT_FAILURE);
  }
  nh_->getParam(LOCATIONS_CONFIG_PARAM, locations_);
  if (!validateYamlType(locations_.getType(), XmlRpc::XmlRpcValue::TypeArray))
  {
    ROS_ERROR("[MoveActionServer]: %s param is invalid (needs to be of an array type) or missing."
              "Make sure that this param exist in locations.yaml and that your launch includes this param file. "
              "shutting down...", LOCATIONS_CONFIG_PARAM);
    ros::shutdown();
    exit(EXIT_FAILURE);
  }
}

/* Parse yaml file and load the locations */
void MoveActionServer::loadLocations()
{
  ROS_INFO("Loading yaml file...");
  /* Create locations_map */
  for (int i = 0; i < locations_.size(); i++)
  {
    // --- validation check ---//
    bool valid = true;
    valid = valid && validateYamlType(locations_[i].getType(), XmlRpc::XmlRpcValue::TypeStruct);
    valid = valid && validateYamlType(locations_[i]["location_name"].getType(), XmlRpc::XmlRpcValue::TypeString);
    valid = valid && validateYamlType(locations_[i]["x"].getType(), XmlRpc::XmlRpcValue::TypeDouble);
    valid = valid && validateYamlType(locations_[i]["y"].getType(), XmlRpc::XmlRpcValue::TypeDouble);
    valid = valid && validateYamlType(locations_[i]["Y"].getType(), XmlRpc::XmlRpcValue::TypeDouble);

    if (!valid)
    {
      ROS_ERROR("[MoveActionServer]: load locations at index %d param data type is invalid or missing."
                "Make sure that this param exist in locations.yaml and that your launch includes "
                "this param file. shutting down...", i);
      ros::shutdown();
      exit(EXIT_FAILURE);
    }
    // ------------------------//

    std::string location_name; // the key
    point p;                   // the value

    location_name = static_cast<std::string>(locations_[i]["location_name"]);
    p.x = static_cast<double>(locations_[i]["x"]);
    p.y = static_cast<double>(locations_[i]["y"]);
    p.Y = static_cast<double>(locations_[i]["Y"]);

    // Add the location to the map
    locations_map_[location_name] = index_counter_++;
    points_vec_.push_back(p);
  }
}

/*
 * The callback function. Called when the client send a goal.
 */
void MoveActionServer::executeCB(const navigation_goal::MoveGoalConstPtr &goal)
{
  ros::Rate r(10);
  bool success = true;

  // Publish info to the console for the user
  std::cout << "Executing, the loaction name is: " << goal->location_name << std::endl;

  if (action_server_->isPreemptRequested() || !ros::ok())
  {
    ROS_INFO("%s: Preempted", action_name_.c_str());
    action_server_->setPreempted();
    success = false;
  }

  if (success)
  {
      if (goal->location_name == "all")
      {
          feedback_.name = "execute all";
          action_server_->publishFeedback(feedback_); // publish the feedback

          result_.res = goal->location_name;
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          action_server_->setSucceeded(result_);

          std::vector<point>::iterator it;
          for (it = points_vec_.begin(); it != points_vec_.end(); ++it)
          {
              cur_point_ = *it;
              std::cout << "sending goal: "
                        << std::endl ;

              // send the goal to move base.
              publishGoal();

              ROS_INFO("waiting in place for 10 sec.");
              ros::Duration(10).sleep();
              ROS_INFO("finish waiting");
          }
      } else
          {
          bool location_name_found = locations_map_.find(goal->location_name) != locations_map_.end();
          if (!location_name_found) // Check if the client's goal exists. If not, exit.
          {
              ROS_ERROR("[MoveActionServer]: couldn't locate model specification for location name %s. "
                        "Make sure locations.yaml contains all the necessary locations. shutting down...",
                        goal->location_name.c_str());
              ros::shutdown();
              exit(EXIT_FAILURE);
          }
          int cur_index = locations_map_[goal->location_name];
          cur_point_ = points_vec_.at((unsigned) cur_index);
          ROS_INFO("x: %f, y: %f, :y %f ", cur_point_.x, cur_point_.y, cur_point_.Y);

          feedback_.name = "found";
          action_server_->publishFeedback(feedback_); // publish the feedback

          result_.res = goal->location_name;
          ROS_INFO("%s: Succeeded", action_name_.c_str());
          action_server_->setSucceeded(result_);

          // send the goal to move base.
          publishGoal();
      }
  }
}


void MoveActionServer::createGoalToMoveBase()
{
    // 'frame_id' is the coordinate system to which we send the goal.
    // If we select "base_link" instead of "map", each time the goal will be in relative to the robot.
    goal_.target_pose.header.frame_id = "map";
    goal_.target_pose.header.stamp = ros::Time::now();

    // z will be 0 by default.
    goal_.target_pose.pose.position.x = cur_point_.x;
    goal_.target_pose.pose.position.y = cur_point_.y;
    //  goal.target_pose.pose.orientation.w = 1.0;

    // Convert the Euler angle to quaternion
    double radians_ = cur_point_.Y * (M_PI / 180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians_); // Create this quaternion from yaw (in radians)
    ROS_INFO_STREAM(quaternion);

    geometry_msgs::Quaternion quaternion_msg_;
    tf::quaternionTFToMsg(quaternion, quaternion_msg_);

    goal_.target_pose.pose.orientation = quaternion_msg_;
}

/* Send a goal to the robot to move to a specific location */
void MoveActionServer::publishGoal()
{
    createGoalToMoveBase(); // create the goal massage
    ROS_INFO("Sending goal: x = %f, y = %f, Y = %f", cur_point_.x, cur_point_.y, cur_point_.Y);
    move_base_client_->sendGoal(goal_); //  push the goal out over the wire to the move_base node for processing.

    // Wait for the action to return
    move_base_client_->waitForResult();

    if (move_base_client_->getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("You have reached the goal!");  // according to move_base.
    }
    else
    {
        ROS_INFO("The base failed for some reason");
    }
}