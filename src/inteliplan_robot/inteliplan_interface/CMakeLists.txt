cmake_minimum_required(VERSION 3.0.2)
project(inteliplan_interface)
find_package(catkin REQUIRED)
# catkin_python_setup()

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  roscpp
  rospy
  std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)