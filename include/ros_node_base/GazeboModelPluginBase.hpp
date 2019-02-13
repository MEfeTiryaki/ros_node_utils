/*
 File name: GazeboModelPluginBase.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 29.01.2019
 Date last modified: 12.02.2019
 */
#pragma once

// c++
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <atomic>
#include <condition_variable>
#include <vector>
#include <math.h>
// required for std::this_thread
#include <thread>

// gazebo
#include <gazebo/gazebo.hh>
#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <sdf/Element.hh>
// ros
#include <ros/ros.h>
#include <ros/callback_queue.h>

// urdf
#include <urdf/model.h>

#include "ros_node_base/RosNodeBase.hpp"

namespace gazebo {

/*! \~english
 This class is a Gazebo plugin base class
 */
class GazeboModelPluginBase : public ModelPlugin, public ros_node_utils::RosNodeBase
{
 public:
  /*! \~english
   Constructor
   */
  GazeboModelPluginBase();

  /*! \~english
   Destructor
   */
  virtual ~GazeboModelPluginBase();

  /*! \~english
   Implements Gazebo virtual load function.
   */
  virtual void Load(physics::ModelPtr model, sdf::ElementPtr sdf /*_sdf*/);

  /*! \~english
   * Overrides Gazebo init function.
   */
  virtual void Init();

  /*! \~english
   * Overrides Gazebo reset function.
   */
  //
  virtual void Reset();

  /*! \~english
   */
  virtual void OnUpdate();

 protected:

  /*! \~english
   * Reads parameters from the parameter server and sdf element.
   */
  virtual void readParameters(sdf::ElementPtr sdf);

  /*! \~english
   */
  virtual void initializeJointStructures();

  /*! \~english
   */
  virtual void initializeLinkStructure();

  /*! \~english
   * Read simulation state.
   */
  virtual void readSimulation();

  /*! \~english
   * Writes simulation state.
   */
  virtual void writeSimulation();

  /*! \~english
   * Publishes robot position, orientation and velocities.
   */
  virtual void publish();

  /*! \~english
   * Publish Transforms of the robot
   */
  virtual void publishTf();

  // Ensures gazebo methods are called sequentially
  std::recursive_mutex gazeboMutex_;

  // Model.
  physics::ModelPtr model_;
  // World update event.
  event::ConnectionPtr updateConnection_;

};
}

