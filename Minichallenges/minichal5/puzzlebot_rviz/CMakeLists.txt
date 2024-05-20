cmake_minimum_required(VERSION 3.0.2)
project(puzzlebot_rviz)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  message_generation
  roscpp
  rospy
  std_msgs
  urdf
  xacro
)

catkin_package(CATKIN_DEPENDS
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

find_package(roslaunch)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
