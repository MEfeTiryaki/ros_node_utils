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

#include "ros_node_utils/RosNodeBase.hpp"

namespace ros_node_utils {

class RosNodeModuleBase : public RosNodeBase
{
 public:
  RosNodeModuleBase(ros::NodeHandle* nodeHandle)
      : RosNodeBase()
  {
    this->nodeHandle_ = nodeHandle;
  }
  ;
  virtual ~RosNodeModuleBase()
  {
  }
  ;


};
}  // namespace ros_node_utils
