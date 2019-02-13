/*
 File name: GazeboModelPluginBase.cpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 12.02.2019
 Date last modified: 12.02.2019
 */


#include "ros_node_base/GazeboModelPluginBase.hpp"

using namespace gazebo;


GazeboModelPluginBase::GazeboModelPluginBase()
    : ros_node_utils::RosNodeBase()
{

}
;

GazeboModelPluginBase::~GazeboModelPluginBase()
{

}


void GazeboModelPluginBase::Load(physics::ModelPtr model, sdf::ElementPtr sdf /*_sdf*/)
{

  nodeHandle_ = new ros::NodeHandle("~");

  // Model
  this->model_ = model;

  create();

  // Note : check if this is placed correctly
  readParameters(sdf);

  // initialize joint structure
  initializeJointStructures();
  initializeLinkStructure();


  initialize();
  // initialize ROS pub/sub/services
  initializeSubscribers();
  initializePublishers();
  initializeServices();

  // reset simulation variables
  Reset();
}


void GazeboModelPluginBase::Init(){}

void GazeboModelPluginBase::Reset(){}

void GazeboModelPluginBase::OnUpdate()
{
  readSimulation();
  writeSimulation();
  publish();
  publishTf();
}

void GazeboModelPluginBase::readParameters(sdf::ElementPtr sdf)
{
}


void GazeboModelPluginBase::initializeJointStructures()
{
}


void GazeboModelPluginBase::initializeLinkStructure()
{
}

void GazeboModelPluginBase::readSimulation()
{
}


void GazeboModelPluginBase::writeSimulation()
{
}
void GazeboModelPluginBase::publish()
{
}

void GazeboModelPluginBase::publishTf()
{
}

