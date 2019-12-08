

#include "ros_node_utils/RosNodeLauncher.hpp"
#include "ros_node_utils/ClockServer.hpp"


int main(int argc, char **argv)
{
  ros_node_utils::RosNodeLauncher<ClockServer>* nodeLauncher =
      new ros_node_utils::RosNodeLauncher<ClockServer>  (ros_node_utils::ROSINITIALIZE("clock_server")) ;
  nodeLauncher->run();
  return 0;
}
