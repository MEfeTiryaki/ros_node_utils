/*
 File name: RosNodeLauncher.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 24.06.2019
 Date last modified: 24.06.2019
 */
#pragma once

#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <functional>

#include <std_srvs/Empty.h>
#include <ros_node_utils/RosNodeBase.hpp>
#include <ros_node_utils/RosNodeModuleBase.hpp>

namespace ros_node_utils {

ros::NodeHandle* ROSINITIALIZE(std::string nodeName)
{
  int argc = 0;
  char **argv = { };
  ros::init(argc, argv, nodeName);
  return new ros::NodeHandle("~");
}
;

template<typename NodeType>
class RosNodeLauncher : public RosNodeModuleBase
{
 public:
  RosNodeLauncher(ros::NodeHandle* nodeHandle)
      : RosNodeModuleBase(nodeHandle)
  {
    create();
    readParameters();
    initializePublishers();
    initializeSubscribers();
    initializeServices();
    initialize();
    node_->start();

  }

  ~RosNodeLauncher()
  {
  }

  // Create the objects in this class
  void create() override
  {
    RosNodeModuleBase::create();
    node_ = std::unique_ptr<NodeType>(new NodeType(getNodeHandle()));
    node_->create();
    //WARNING("create : [RosNodeLauncher]");

  }

  // Reading parameters
  void readParameters() override
  {
    RosNodeModuleBase::readParameters();
    node_->readParameters();
    //WARNING("readParameters : [RosNodeLauncher]");
  }

  // initize class variables
  void initialize() override
  {
    RosNodeModuleBase::initialize();
    node_->initialize();
    //WARNING("initialize : [RosNodeLauncher]");
  }

  // shutdown class variables
  void shutdown() override
  {
    RosNodeModuleBase::shutdown();
    node_->shutdown();
    //WARNING("shutdown : [RosNodeLauncher]");
  }

  // init Publisher
  void initializePublishers() override
  {
    RosNodeModuleBase::initializePublishers();
    node_->initializePublishers();
    //WARNING("initializePublishers : [RosNodeLauncher]");
  }

  // init Subscribers
  void initializeSubscribers() override
  {
    RosNodeModuleBase::initializeSubscribers();
    node_->initializeSubscribers();
    //WARNING("initializeSubscribers : [RosNodeLauncher]");
  }

  // init Subscribers
  void initializeServices() override
  {
    RosNodeModuleBase::initializeServices();
    node_->initializeServices();
    nodeRestartServices_ = this->nodeHandle_->advertiseService(
        "/ros_node_launch/restart", &RosNodeLauncher::nodeRestartCallback, this);
    //WARNING("initializeServices : [RosNodeLauncher]");
  }

  void run()
  {
    ros::Rate mainRate = ros::Rate(100);
    while (ros::ok()) {
      ros::spinOnce();
      mainRate.sleep();
    }
  }

  void restart()
  {
    node_->shutdown();
    node_ = std::unique_ptr<NodeType>(new NodeType(getNodeHandle()));
    node_->create();
    node_->readParameters();
    node_->initializePublishers();
    node_->initializeSubscribers();
    node_->initializeServices();
    node_->initialize();
    node_->start();
  }

  bool nodeRestartCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
  {
    restart();
    return true;
  }

 protected:

  ros::ServiceServer nodeRestartServices_;

  std::unique_ptr<RosNodeBase> node_;

};
}  // namespace ros_node_utils
