cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_server_msgs)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  geometry_msgs
  genmsg
  roslib
)


add_message_files(
  FILES
  SemanticMapMetaData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs message_runtime roslib
)