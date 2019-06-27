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

#include <ros_node_utils/ros_node_utils.hpp>

namespace ros_node_utils {

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

  virtual ~RosNodeBase()
  {
  }


  // Create the objects in this class
  virtual void create()
  {

  }

  // Reading parameters
  virtual void readParameters()
  {

  }

  // initize class variables
  virtual void initialize()
  {

  }

  // shutdown class variables
  virtual void shutdown()
  {

  }


  // init Publisher
  virtual void initializePublishers()
  {

  }

  // init Subscribers
  virtual void initializeSubscribers()
  {

  }

  // init Services
  virtual void initializeServices()
  {
    //services_.insert(
    //    std::make_pair<std::string, ros::ServiceServer>(
    //        name,
    //        nodeHandle_->advertiseService(namespace_ + "/" + nodeName_ + "/" + name, &Callback,
    //                                      this)));
  }

  // init Services
  virtual void initializeActionServers()
  {

  }


  virtual void execute()
  {

  }

  virtual void start()
  {

  }

  virtual void stop()
  {

  }

  ros::NodeHandle* getNodeHandle(){
    return nodeHandle_;
  }

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
