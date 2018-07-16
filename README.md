# ros_node_utils
The ros_node_utils offers a easy to follow "object oriented programming" scheme for fast prototyping and developing modules integrated to ROS system. ros_node utils offers following options;
* RosNodeBase (cpp/py) : a parent class that contains all essential functions required by a simple ros module
* RosExecuterBase (cpp/py) : a parent class to easily create executable ros module, which potentially contain multiple objects of RosNodeBase drived classes.
* RosTfPublisher(py) : a interface to easily publish constant transformations or to route published poses to transformations for RViZ visualizations. (cpp version will be added with additional functionallity)
* RosParamReader(cpp) : Coming soon. simple interface to assign rosparameters to prefered types (std::vector,eigen::MatrixXd,eigen::VectorXd etc.)  

## System Requirements
ros_node_utils package requires following system specifications; 
* [Ubuntu](https://www.ubuntu.com/) 16.04
* [GCC](https://gcc.gnu.org/) 5.4.0 
* or [Python 2.7]
* [Ros](http://www.ros.org/)-kinetic
* Eigen 3.3 (Ubuntu 16.04 libeigen-dev bulunduruyor)
* Boost 1.58.0
* [catkin_tools](http://catkin-tools.readthedocs.io/en/latest/verbs/catkin_build.html) 
