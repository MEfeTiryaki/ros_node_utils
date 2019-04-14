cmake_minimum_required(VERSION 2.8.3)
project(ros_node_utils)

add_definitions("-std=c++11")

find_package(gazebo REQUIRED)

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
COMPONENTS
  roscpp
  rospy
  gazebo_plugins
  gazebo_ros
  urdf
  xacro
  tf
)

catkin_package(
INCLUDE_DIRS
  include
  ${GAZEBO_INCLUDE_DIRS}
   ${EIGEN3_INCLUDE_DIR}
LIBRARIES
  ${PROJECT_NAME} # If this is missing it can not find GazeboModelPluginBase constructor
CATKIN_DEPENDS
  roscpp
  rospy
  gazebo_plugins
  gazebo_ros
  urdf
  xacro
  tf
)

message(WARNING ${catkin_INCLUDE_DIR})
message(WARNING ${GAZEBO_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}

)

link_directories(
  ${GAZEBO_LIBRARY_DIRS}
)
add_executable(node
  src/ros_node_utils/node.cpp
)
add_dependencies(node
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(node
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)


add_library( ${PROJECT_NAME}
  src/ros_gazebo_utils/GazeboModelPluginBase.cpp
)
add_dependencies(${PROJECT_NAME}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
