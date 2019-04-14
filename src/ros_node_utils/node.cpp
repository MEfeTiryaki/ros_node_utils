#include <iostream>
#include "ros_node_utils/RosExecuterNodeBase.hpp"
#include "ros_node_utils/ros_node_utils.hpp"

int main(int argc, char **argv)
{
  ros_node_utils::RosExecuterNodeBase node_ = ros_node_utils::RosExecuterNodeBase("no_name") ;
  node_.execute();
  ros_node_utils::CONFIRM("node works fine.");
  return 0;
}
