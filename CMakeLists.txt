cmake_minimum_required(VERSION 2.8.3)
project(ros_node_utils)

add_definitions("-std=c++11")

find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES # only header files, therefore no
  CATKIN_DEPENDS
    roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
link_directories(
  include
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

message( ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#        PATTERN ".svn" EXCLUDE
)
