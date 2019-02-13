#include <iostream>
#include "ros_node_base/RosExecuterNodeBase.hpp"

int main(int argc, char **argv)
{
  ros_node_utils::RosExecuterNodeBase node_ = ros_node_utils::RosExecuterNodeBase("no_name") ;
  node_.execute();
  return 0;
}
