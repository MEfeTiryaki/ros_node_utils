cmake_minimum_required(VERSION 2.8.3)
project(ros_node_utils)

add_definitions("-std=c++11")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
COMPONENTS
  roscpp
  rospy
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
    urdf
    xacro
    tf
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(
  include
  ${catkin_LIBRARY_DIRS}
)


install(DIRECTORY include/${PROJECT_NAME}
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        PATTERN ".svn" EXCLUDE
)

# TEST

catkin_add_gtest(utest test/test_trial.cpp)

target_link_libraries(utest
  ${catkin_LIBRARIES}
)

add_executable(node_test
  src/ros_node_utils/node.cpp
)
add_dependencies(node_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(node_test
  ${catkin_LIBRARIES}
)


add_executable(clock_server
  src/ros_node_utils/clock_server.cpp
)
add_dependencies(clock_server
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(clock_server
  ${catkin_LIBRARIES}
)


install(TARGETS node_test
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})
