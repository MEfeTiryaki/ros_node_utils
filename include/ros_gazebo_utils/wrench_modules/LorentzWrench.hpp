/*
 File name: LorentzWrench.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: tiryaki@is.mpg.de or m.efetiryaki@gmail.com
 Date created: 14.04.2019
 Date last modified: 15.04.2019
 */

#pragma once

// c++
#include <mutex>
#include <string>
#include <vector>
#include <math.h>

// ROS Messages

#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Float64.h>

#include "ros_gazebo_utils/wrench_modules/WrenchModuleBase.hpp"

using namespace ros_node_utils;

namespace gazebo {
namespace wrench {

class LorentzWrench : public WrenchModuleBase
{
 public:
  LorentzWrench(ros::NodeHandle* nodeHandle, wrench::WrenchLink* link)
      : WrenchModuleBase(nodeHandle, link),
  B_0_(Eigen::Vector3d::Zero()),
  B_0_direction_(Eigen::Vector3d(1.0, 0.0, 0.0)),
  current_(0.0)
  {
    this->name_ = "Lorenz_force";
  }
  ;

  ;
  virtual ~LorentzWrench()
  {
  }
  ;

  virtual void readParameters() override
  {
    paramRead(this->nodeHandle_, "/physics/magnetic/B_0", B_0_);

  }

  virtual void initialize() override
  {
    if (B_0_.norm() == 0) {
      B_0_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
      B_0_direction_ = B_0_.normalized();
    }
  }

  virtual void initializeSubscribers() override
  {
    magneticFieldSubscriber_ = this->nodeHandle_->subscribe("magnetic_field", 10,
                                                            &LorentzWrench::magneticFieldCallback,
                                                            this);
    currentSubscriber_ = this->nodeHandle_->subscribe("coil_current", 10,
                                                      &LorentzWrench::currentCallback, this);
  }

  Eigen::Vector3d getMainMagneticField()
  {
    return B_0_;
  }

  void setMainMagneticField(Eigen::Vector3d B_0)
  {
    B_0_ = B_0;
    if (B_0_.norm() == 0) {
      B_0_direction_ = Eigen::Vector3d(1.0, 0.0, 0.0);
    } else {
      B_0_direction_ = B_0_.normalized();
    }
  }
  ;


  //CALLBACK
  void magneticFieldCallback(const std_msgs::Float64MultiArray& msg)
  {
    if (msg.data.size() == 3) {
      for (int i = 0; i < msg.data.size(); i++) {
        B_0_[i] = msg.data[i];
      }
    } else {
      //std::cerr << "\033[0;31m" << "Magnetic Gradient should be 3D" << "\033[0m" << std::endl;
    }
  }
  ;

  void currentCallback(const std_msgs::Float64& msg)
  {
    current_ = msg.data;
  }
  ;

 protected:
  Eigen::Vector3d B_0_;
  Eigen::Vector3d B_0_direction_;
  double current_;

  // Subscriber
  ros::Subscriber magneticFieldSubscriber_;
  ros::Subscriber currentSubscriber_;
};
}  // namespace wrench
}  // namespace gazebo
