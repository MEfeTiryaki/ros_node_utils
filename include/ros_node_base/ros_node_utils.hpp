/*@file
 File name: ros_node_utils.hpp
 Author: Mehmet Efe Tiryaki
 E-mail: m.efetiryaki@gmail.com
 Date created: 12.02.2019
 Date last modified: 21.03.2019
 */
#pragma once

#include <ros/ros.h>
#include <vector>
#include <string>
#include <unordered_map>
#include <functional>
#include <Eigen/Dense>

namespace ros_node_utils {

inline bool ERROR(std::string text)
{
  std::cout << "\033[0;91m" << text << "\033[0m" << std::endl;
  return false;
}

inline bool WARNING(std::string text)
{
  std::cout << "\033[0;93m" << text << "\033[0m" << std::endl;
  return true;
}

inline bool CONFIRM(std::string text)
{
  std::cout << "\033[0;92m" << text << "\033[0m" << std::endl;
  return true;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, bool& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, variable);
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;
  //std::cerr << "\033[0;31m" << variable << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, int& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, variable);
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;
  //std::cerr << "\033[0;31m" << variable << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, double& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, variable);
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;
  //std::cerr << "\033[0;31m" << variable << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, std::string& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, variable);
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;
  //std::cerr << "\033[0;31m" << variable << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName,
                      std::vector<int>& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, variable);
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName,
                      std::vector<double>& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, variable);
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;

}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName,
                      std::vector<std::string>& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, variable);
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;

}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, Eigen::MatrixXd& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  double* ptr;
  std::vector<double> buffer;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, buffer);
    ptr = &buffer[0];
    variable.diagonal() << Eigen::Map<Eigen::VectorXd>(ptr, buffer.size());
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;

  //std::cerr << "\033[0;31m" << variable << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, Eigen::MatrixXd& variable,
                      int row)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  double* ptr;
  std::vector<double> buffer;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, buffer);
    int col = buffer.size() / row;
    variable = Eigen::MatrixXd::Zero(row, col);
    for (int i = 0; i < buffer.size(); i++) {
      variable(i / col, i % col) = buffer[i];
    }
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;
  //std::cerr << "\033[0;31m" << variable << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, Eigen::VectorXd& variable)
{
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  int prevSize = variable.size();
  std::vector<double> buffer;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, buffer);
    variable = Eigen::VectorXd::Zero(buffer.size());
    for (int i = 0; i < buffer.size(); i++) {
      variable[i] = buffer[i];
    }
  } else {
    return ERROR(paramName + " is not found. ");
  }
  if (prevSize != variable.size()) {
    return WARNING(
        paramName + " has changed the vector size from " + std::to_string(prevSize) + " to "
            + std::to_string(variable.size()));
  }
  return true;

  //std::cerr << "\033[0;31m" << variable.transpose() << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, Eigen::Vector2d& variable)
{
  int size = 2;
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  std::vector<double> buffer;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, buffer);
    variable = Eigen::VectorXd::Zero(size);
    for (int i = 0; i < size; i++) {
      variable[i] = buffer[i];
    }
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;

  //std::cerr << "\033[0;31m" << variable.transpose() << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, Eigen::Vector3d& variable)
{
  int size = 3;
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  std::vector<double> buffer;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, buffer);
    variable = Eigen::VectorXd::Zero(size);
    for (int i = 0; i < size; i++) {
      variable[i] = buffer[i];
    }
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;
  //std::cerr << "\033[0;31m" << variable.transpose() << "\033[0m" << std::endl;
}

inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, Eigen::Vector4d& variable)
{
  int size = 4;
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  std::vector<double> buffer;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, buffer);
    variable = Eigen::VectorXd::Zero(size);
    for (int i = 0; i < size; i++) {
      variable[i] = buffer[i];
    }
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;

  //std::cerr << "\033[0;31m" << variable.transpose() << "\033[0m" << std::endl;
}

// TODO : ADD a Quaternion value check
inline bool paramRead(ros::NodeHandle* NodeHandle, std::string paramName, Eigen::Quaterniond& variable)
{
  int size = 4;
  //std::cerr << "\033[0;32m" << paramName << "\033[0m" << std::endl;
  std::vector<double> buffer;
  if (NodeHandle->hasParam(paramName)) {
    NodeHandle->getParam(paramName, buffer);
    if(buffer.size()==4){
      variable = Eigen::Quaterniond(buffer[0],buffer[1],buffer[2],buffer[3]);
    }else{
      return ERROR(paramName + " is not a Quaternion ");
    }
  } else {
    return ERROR(paramName + " is not found. ");
  }
  return true;

  //std::cerr << "\033[0;31m" << variable.transpose() << "\033[0m" << std::endl;
}

}  // namespace ros_node_utils
