/*
 File name: RosExecuterNodeBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 1.07.2018
 Date last modified: 1.07.2018
 */
#pragma once

#include <ros_node_utils/RosNodeBase.hpp>


namespace ros_node_utils {

class RosExecuterNodeBase : public RosNodeBase
{
 public:
  RosExecuterNodeBase(std::string nodeName):
    RosNodeBase()
  {
    this->nodeName_ = nodeName;
    int argc = 0;
    char **argv = { };
    ros::init(argc, argv, nodeName_);
    namespace_ = ros::this_node::getNamespace();
    namespace_.erase(0, 1);
    this->nodeHandle_ = new ros::NodeHandle("~");
    this->nodeName_ = ros::this_node::getName();
  }
  ;
  virtual ~RosExecuterNodeBase()
  {
  }


  virtual void initialize() override
  {
    initializePublishers();
    // init Subscribers
    initializeSubscribers();
    // init Services
    initializeServices();
    // init Services
    initializeActionServers();
  }

  virtual void execute()
  {
  }



};
}  // namespace ros_node_utils
