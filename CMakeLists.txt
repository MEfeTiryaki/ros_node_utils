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
  LIBRARIES # only header files, therefore no libraries
  CATKIN_DEPENDS
    roscpp
    rospy
    gazebo_plugins
    gazebo_ros
    urdf
    xacro
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(
  include
  ${catkin_LIBRARY_DIRS}
  ${GAZEBO_LIBRARY_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
)

# TEST
add_executable(node_test
  src/ros_node_utils/node.cpp
)
add_dependencies(node_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(node_test
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
)

install(TARGETS node_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
