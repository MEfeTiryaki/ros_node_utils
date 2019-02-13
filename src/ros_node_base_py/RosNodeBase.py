'''
    File name: RosNodeBase.py
    Author: Mehmet Efe Tiryaki
    E-mail: m.efetiryaki@gmail.com
    Date created: 23.05.2018
    Date last modified: 23.05.2018
    Python Version: 2.7
'''
import sys
sys.dont_write_bytecode = True


import rospy

class RosNodeBase(object):
    """ This class defines minimum class structure for a rosnode
    """

    def __init__(self):
        """ Set default initilization procedure for the ros node. Once you inherent from this class, YOU SHOULD NOT NEED to implement __init__ in child class anymore.
        """
        self.create()
        # Reading parameters
        self.readParameters()
        # ROS Node initilization
        rospy.init_node("~")
        # initize class variables
        self.initialize()
        # init Publisher
        self.initializePublishers()
        # init Subscribers
        self.initializeSubscribers()
        # init Services
        self.initializeServices()
        # Start estimator
        self.execute()


    def __str__(self):
        """ The print output of the class, useful for debugging
        """
        return "RosNodeBase string "
    def create(self):
        """ Create all of the instance variables here.
        """
        # namespace of the node
        self.ns_ = rospy.get_namespace()
    def initialize(self):
        """ initialize the necessary objects here and call other initilization methods for your instance variables if there is any.
        """
        pass
    def readParameters(self):
        """ Read ROS parameters and any other input file here
        """
        pass
    def initializePublishers(self):
        """ initialize all publishers here
        """
        pass
    def initializeSubscribers(self):
        """ initialize all subscribers here
        """
        pass
    def initializeServices(self):
        """ initialize all services here
        """
        pass
    def execute(self):
        """ Depending on your node's purpose, insert here a LOOP or One time call
        """
        pass
    def advance(self):
        """ The jobs, which you are doing at every cycle of the node should be here. This is the main method you should develop. For nodes with multiple subscribers and services use thread lock
        """
        pass
