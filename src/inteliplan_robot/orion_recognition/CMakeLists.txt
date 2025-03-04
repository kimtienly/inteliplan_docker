cmake_minimum_required(VERSION 2.8.3)
project(orion_recognition)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_filters
  roscpp
  roslib
  sensor_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_generation
  orion_actions
  visualization_msgs
)

catkin_python_setup()

generate_messages(
  DEPENDENCIES
  actionlib_msgs
  std_msgs
  visualization_msgs
)

catkin_package(
  CATKIN_DEPENDS
  cv_bridge
  image_transport
  roscpp
  roslib
  sensor_msgs
  std_msgs
  geometry_msgs
  actionlib_msgs
  message_runtime
  orion_actions
)

include_directories(
  SYSTEM ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  include
)

catkin_install_python(PROGRAMS
  scripts/check_for_object_server_node.py
  scripts/bbox_publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
