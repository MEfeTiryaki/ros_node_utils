/*
 File name: RosNodeModuleBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 12.02.2018
 Date last modified: 13.02.2019
 */

#pragma once

#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <functional>
#include <mutex>


#include "ros_node_utils/RosNodeBase.hpp"

namespace ros_node_utils {

class RosNodeModuleBase : public RosNodeBase
{
 public:
  RosNodeModuleBase(ros::NodeHandle* nodeHandle)
      : RosNodeBase(),
        shutdownMutex_(new std::mutex()),
        terminated_(false),
        terminate_(false)
  {
    this->nodeHandle_ = nodeHandle;
    this->nodeName_ = ros::this_node::getName();
  }

  virtual ~RosNodeModuleBase()
  {
  }

  virtual void create() override
  {
    RosNodeBase::create();
    terminated_ = false;
    terminate_ = false;
  }

  virtual void shutdown() override
  {
    terminate_ = true;
  }

  virtual void clean(){

  }

  bool isTerminated()
  {
    return terminated_;
  }

  void terminate()
  {
    terminated_ = true;
  }

  bool isTerminationStarted()
  {
    return terminate_;
  }

 protected:
  std::mutex* shutdownMutex_;
  bool terminated_;
  bool terminate_;
};
}  // namespace ros_node_utils
