/*
 File name: ClockServer.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 19.06.2018
 Date last modified: 13.02.2019
 */
#pragma once


#include <boost/thread.hpp>
#include <boost/chrono.hpp>
#include <math.h>
#include <memory>
#include <mutex>

#include "rosgraph_msgs/Clock.h"

class ClockServer : public ros_node_utils::RosNodeModuleBase
{
 public:
  ClockServer(ros::NodeHandle* nodeHandle)
      : ros_node_utils::RosNodeModuleBase(nodeHandle),
      startTime_(0)
  {
  }

  virtual ~ClockServer()
  {

  }

  void create() override
  {
    RosNodeModuleBase::create();
    clockRate_ = 1000;
    this->rate_ = new ros::Rate(clockRate_);

    startTime_ = ros::Time::now().toSec();
  }

  void initializePublishers(){
    clockPublisher_ =
        this->nodeHandle_->advertise<rosgraph_msgs::Clock>(
            "/clock", 10);
  }

  void advance()
  {
    timeNow_ =   ros::Time::now().toSec() - startTime_;
    rosgraph_msgs::Clock msg = rosgraph_msgs::Clock();
    msg.clock = ros::Time(timeNow_);
    clockPublisher_.publish(msg);
  }

  void execute()
  {
    while (ros::ok()) {
    {
      advance();
    }
    this->rate_->sleep();
  }
  }

  void start() override
  {
    timeThread_ = new boost::thread(boost::bind(&ClockServer::execute, this));
  }

  virtual void stop()
  {
    timeThread_->detach();
  }

 protected:
   boost::thread* timeThread_;

  double startTime_ ;
  double timeNow_ ;
  double clockRate_;


  ros::Publisher clockPublisher_;
};
