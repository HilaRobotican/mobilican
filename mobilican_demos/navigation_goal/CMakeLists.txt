cmake_minimum_required(VERSION 2.8.3)
project(navigation_goal)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(
    catkin REQUIRED COMPONENTS 
    roscpp
    rospy
    actionlib 
    actionlib_msgs
    actionlib
    move_base_msgs
    genmsg 
    sensor_msgs
    std_msgs
    message_generation
)

find_package(Boost REQUIRED COMPONENTS)

find_package(OpenCV) 

# Generate services in the 'srv' folder
add_service_files(
   FILES
   ImageSnapshot.srv
)

## Add actions
add_action_files(
    DIRECTORY action
    FILES Move.action)

## Generate messages
generate_messages(DEPENDENCIES actionlib_msgs std_msgs)

## Define catkin exports
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS actionlib_msgs cv_bridge opencv2 roscpp rospy sensor_msgs std_msgs message_runtime
  DEPENDS Boost
)

## Setup include dirs
include_directories(
 include
 ${catkin_INCLUDE_DIRS} ${yaml-cpp_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS}
 )


add_executable(move_server_node
		src/move_server_node.cpp
        src/move_action_server.cpp
        include/move_action_server.h
		include/point.h)

target_link_libraries(
	move_server_node
	${catkin_LIBRARIES}
)

add_dependencies(move_server_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_executable(move_client_node
		src/move_client_node.cpp
		src/move_action_client.cpp
		include/move_action_client.h)

target_link_libraries(
	move_client_node
	${catkin_LIBRARIES}
)

add_dependencies(move_client_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

        

