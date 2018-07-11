cmake_minimum_required(VERSION 2.8.3)
project(ros_node_utils)

add_definitions("-std=c++11")

find_package(catkin REQUIRED
  COMPONENTS
    rospy
    roscpp
)

catkin_python_setup()

catkin_package(
INCLUDE_DIRS
  include
LIBRARIES
CATKIN_DEPENDS
  roscpp
  rospy
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(node
  src/ros_node_base/node.cpp
)
add_dependencies(node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(node
  ${catkin_LIBRARIES}
)
