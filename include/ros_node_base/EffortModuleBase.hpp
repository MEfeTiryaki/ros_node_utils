/*
 File name: EffortModuleBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: tiryaki@is.mpg.de or m.efetiryaki@gmail.com
 Date created: 25.01.2019
 Date last modified: 17.03.2019
 */
//TODO EFE TIRYAKI 21.03.2019 : separate the visulization part
#pragma once

// c++
#include <mutex>
#include <string>
#include <vector>
#include <math.h>

#include <Eigen/Dense>

#include "ros_node_base/RosNodeModuleBase.hpp"

namespace gazebo {
namespace effort {

class EffortModuleBase : public ros_node_utils::RosNodeModuleBase
{
 public:
  EffortModuleBase(ros::NodeHandle* nodeHandle)
      : ros_node_utils::RosNodeModuleBase(nodeHandle),
        scale_(1.0)
  {

  }
  ;

  virtual ~EffortModuleBase()
  {
  }
  ;

  virtual void initialize() override
  {
    ros_node_utils::RosNodeModuleBase::initialize();
  }
  ;

  virtual void setName(std::string name)
  {
    name_ = name;
  }
  ;
  virtual std::string getName()
  {
    return name_;
  }
  ;

  virtual void setOrigin(Eigen::Vector3d origin)
  {
    origin_ = origin;
  }
  ;
  virtual Eigen::Vector3d getOrigin()
  {
    return origin_;
  }
  ;

  virtual void setPosition(Eigen::Vector3d position)
  {
    position_ = position;
  }
  ;

  virtual Eigen::Vector3d getPosition()
  {
    return position_;
  }
  ;

  virtual void setOrientation(Eigen::Quaterniond orientation)
  {
    orientation_ = orientation;
  }
  ;

  virtual Eigen::Quaterniond getOrientation()
  {
    return orientation_;
  }
  ;

  virtual void setLinearVelocity(Eigen::Vector3d linearVelocity)
  {
    linearVelocity_ = linearVelocity;
  }
  ;

  virtual Eigen::Vector3d getLinearVelocity()
  {
    return linearVelocity_;
  }
  ;

  virtual void setAngularVelocity(Eigen::Vector3d angularVelocity)
  {
    angularVelocity_ = angularVelocity;
  }
  ;
  virtual Eigen::Vector3d getAngularVelocity()
  {
    return angularVelocity_;
  }
  ;

  virtual Eigen::Vector3d getForce()
  {
    return Eigen::Vector3d::Zero();
  }
  ;

  virtual Eigen::Vector3d getTorque()
  {
    return Eigen::Vector3d::Zero();
  }
  ;

  virtual void setForceColor(Eigen::Vector4d color)
  {
    forceColor_ = color;
  }
  ;
  virtual Eigen::Vector4d getForceColor()
  {
    return forceColor_;
  }
  ;
  virtual void setTorqueColor(Eigen::Vector4d color)
  {
    torqueColor_ = color;
  }
  ;
  virtual Eigen::Vector4d getTorqueColor()
  {
    return torqueColor_;
  }
  ;
  virtual void setScale(double scale)
  {
    scale_ = scale;
  }
  ;

  virtual double getScale()
  {
    return scale_;
  }
  ;

 protected:
  std::string name_;

  Eigen::Vector3d origin_;
  Eigen::Vector3d force_;
  Eigen::Vector3d torque_;

  Eigen::Vector3d position_;
  Eigen::Quaterniond orientation_;

  Eigen::Vector3d linearVelocity_;
  Eigen::Vector3d angularVelocity_;

  // visulization information
  Eigen::Vector4d forceColor_;
  Eigen::Vector4d torqueColor_;
  double scale_;
};
}  // namespace effort
}  // namespace gazebo
