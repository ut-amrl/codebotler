cmake_minimum_required(VERSION 3.0.2)
project(robot_actions_pkg)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  actionlib_msgs
)

add_action_files(
  FILES
  Ask.action
  GetAllRooms.action
  GetCurrentLocation.action
  GoTo.action
  IsInRoom.action
  Say.action
  Execute.action
)

generate_messages(
  DEPENDENCIES
  std_msgs
  actionlib_msgs
)

catkin_package(
 CATKIN_DEPENDS rospy std_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
