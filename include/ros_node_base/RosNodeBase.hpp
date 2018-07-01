/*
 File name: RosNodeBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 1.07.2018
 Date last modified: 1.07.2018
 */
#pragma once

#include <ros/ros.h>
#include <vector>
#include <unordered_map>
#include <functional>

namespace ros_node_base {

class RosNodeBase

{
 public:
  RosNodeBase()
      : nodeHandle_(),
        rate_()
  {
    namespace_ = ros::this_node::getNamespace();
    namespace_.erase(0, 1);
  }
  ;
  virtual ~RosNodeBase()
  {
  }
  ;

  virtual void execute()
  {

  }
  ;

  // Create the objects in this class
  virtual void create()
  {

  }
  ;
  // Reading parameters
  virtual void readParameters()
  {

  }
  ;
  // initize class variables
  virtual void initilize()
  {

  }
  ;
  // init Publisher
  virtual void initilizePublishers()
  {

  }
  ;
  // init Subscribers
  virtual void initilizeSubscribers()
  {

  }
  ;
  // init Services
  virtual void initilizeServices()
  {
    //services_.insert(
    //    std::make_pair<std::string, ros::ServiceServer>(
    //        name,
    //        nodeHandle_->advertiseService(namespace_ + "/" + nodeName_ + "/" + name, &Callback,
    //                                      this)));
  }
  ;
  // init Services
  virtual void initilizeActionServers()
  {

  }
  ;

 protected:
  std::string namespace_;
  std::string nodeName_;
  // nod handle pointer which can be shared with modules declared in this class
  ros::NodeHandle* nodeHandle_;
  ros::Rate* rate_;

  // Publisher
  std::vector<std::string> publisherNames_;
  std::unordered_map<std::string, ros::Publisher> publishers_;
  std::unordered_map<std::string, int> publisherQueueSizes_;
  // Subscribers
  std::vector<std::string> subscriberNames_;
  std::unordered_map<std::string, ros::Subscriber> subscribers_;
  //std::unordered_map<std::string,ros::ServiceServer> subscribeCallback_;
  // Services
  std::vector<std::string> serviceNames_;
  std::unordered_map<std::string, ros::ServiceServer> services_;
  //std::unordered_map<std::string,auto> serviceCallbacks_;
  // Action Servers
  //

};
}  // namespace ros_node_utils
