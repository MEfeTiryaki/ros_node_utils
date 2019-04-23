

#include <ros/ros.h>
#include <vector>
#include <Eigen/Dense>
#include "ros_gazebo_utils/wrench_modules/MagneticWrench.hpp"
#include "ros_gazebo_utils/WrenchLink.hpp"

#include <gtest/gtest.h>

using namespace gazebo;
using namespace wrench;

TEST(TestSuite, testCase1)
{
  int argc = 0;
  char **argv = { };
  ros::init(argc, argv, "wrench_module_test");
  ros::NodeHandle* nodeHandle_ = new ros::NodeHandle("~");
  // Create Objects
  WrenchLink* wrenchLink_ = new WrenchLink();
  MagneticWrench* magneticWrench = new MagneticWrench(nodeHandle_,wrenchLink_);

  magneticWrench->create();
  magneticWrench->readParameters();
  magneticWrench->initialize();
  magneticWrench->initializeSubscribers();
  magneticWrench->initializePublishers();
  magneticWrench->initializeServices();


  Eigen::Vector3d positionWorldToBase;
  Eigen::Quaterniond orientationWorldToBase;
  Eigen::Vector3d linearVelocityOfBaseInBaseFrame;
  Eigen::Vector3d angularVelocityOfBaseInBaseFrame;
  paramRead(nodeHandle_, "/magnetic_wrench_test/position", positionWorldToBase);
  paramRead(nodeHandle_, "/magnetic_wrench_test/orientation", orientationWorldToBase);
  paramRead(nodeHandle_, "/magnetic_wrench_test/linear_velocity", linearVelocityOfBaseInBaseFrame);
  paramRead(nodeHandle_, "/magnetic_wrench_test/angular_velocity", angularVelocityOfBaseInBaseFrame);
  wrenchLink_->setPositionWorldtoBase(positionWorldToBase);
  wrenchLink_->setOrientationWorldtoBase(orientationWorldToBase);
  wrenchLink_->setLinearVelocityOfBaseInBaseFrame(linearVelocityOfBaseInBaseFrame);
  wrenchLink_->setAngularVelocityOfBaseInBaseFrame(angularVelocityOfBaseInBaseFrame);

  EXPECT_EQ( wrenchLink_->getPositionWorldtoBase(),Eigen::Vector3d(1.0, 0.0, 0.0));
  //EXPECT_EQ( wrenchLink_->getOrientationWorldtoBase(),Eigen::Quaterniond(1.0, 0.0, 0.0, 0.0));
  EXPECT_EQ( wrenchLink_->getLinearVelocityOfBaseInBaseFrame(),Eigen::Vector3d(0.0, 0.0, 0.0));
  EXPECT_EQ( wrenchLink_->getAngularVelocityOfBaseInBaseFrame(),Eigen::Vector3d(0.0, 0.0, 0.0));


  // CALCULATE AEROYNAMICS
  magneticWrench->advance();
  Eigen::Vector3d origin = magneticWrench->getOrigin();
  Eigen::Vector3d forceInWorldFrame = magneticWrench->getForceInWorldFrame();
  Eigen::Vector3d torqueInWorldFrame = magneticWrench->getTorqueInWorldFrame();

  EXPECT_EQ( magneticWrench->getForceInWorldFrame(),Eigen::Vector3d::Zero());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  ros::init(argc, argv, "wrench_module_test");
  ros::NodeHandle* nodeHandle_ = new ros::NodeHandle("~");
  // SET TEST PARAMETERS

  nodeHandle_->setParam("/magnetic_wrench_test/position", std::vector<double>({1.0, 0.0, 0.0}));
  nodeHandle_->setParam("/magnetic_wrench_test/orientation", std::vector<double>({1.0, 0.0, 0.0,0.0}));
  nodeHandle_->setParam("/magnetic_wrench_test/linear_velocity", std::vector<double>({0.0, 0.0, 0.0}));
  nodeHandle_->setParam("/magnetic_wrench_test/angular_velocity", std::vector<double>({0.0, 0.0, 0.0}));
  nodeHandle_->setParam("/physics/magnetic/magnetization", 1);
  nodeHandle_->setParam("/physics/magnetic/magnet_size", 0.1);



  return RUN_ALL_TESTS();






}
