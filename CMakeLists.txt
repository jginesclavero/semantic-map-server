cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  costmap_2d
  geometry_msgs
  genmsg
  roslib
  roslint
  tf2_geometry_msgs
  tf2
  tf2_ros
)

roslint_cpp()

add_message_files(
  FILES
  SemanticMapMetaData.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp std_msgs message_runtime roslib

)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(semantic_map_server_node src/semantic_map_server_node.cpp)
add_dependencies(semantic_map_server_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_map_server_node ${catkin_LIBRARIES})

add_executable(semantic_watcher_node src/semantic_watcher_node.cpp)
add_dependencies(semantic_watcher_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_watcher_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS
  semantic_map_server_node
  semantic_watcher_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
