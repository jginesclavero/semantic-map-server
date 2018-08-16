cmake_minimum_required(VERSION 2.8.3)
project(semantic_map_server)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  message_generation
  costmap_2d
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
)

include_directories(
	include
  ${catkin_INCLUDE_DIRS}
)

add_executable(semantic_map_server_node src/semantic_map_server_node.cpp)
add_dependencies(semantic_map_server_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(semantic_map_server_node ${catkin_LIBRARIES})

## Mark executables and/or libraries for installation
install(TARGETS
  semantic_map_server_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
