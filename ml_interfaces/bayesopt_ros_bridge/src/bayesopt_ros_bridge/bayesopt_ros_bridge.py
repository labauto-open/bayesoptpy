#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from time import sleep
from .bayesopt_ros_bridge_publisher import BayesoptROSBridgePublisher as Publisher
from .bayesopt_ros_bridge_subscriber import BayesoptROSBridgeSubscriber as Subscriber


class BayesoptROSBridge(object):
    def __init__(self, bayesopt_interface, config, pub_rate, data_save_dir):
        self.bayesopt_interface = bayesopt_interface
        self.config = config
        self.pub_rate = pub_rate
        self.data_save_dir = data_save_dir

        self.timer = None

        self.sub = None
        self.pub = None

        self.load_config(self.config)


    def load_config(self, config):
        # subscriber
        rospy.loginfo("Loading result config")
        self.sub = Subscriber(config, self.bayesopt_interface, self.data_save_dir)

        # publisher
        rospy.loginfo("Loading next param config")
        self.pub = Publisher(config, self.bayesopt_interface)


    def run(self):
        # timer enables additional loop with constant period
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.pub_rate), self.update)


    def update(self, event):
        # check search completed
        self.pub.publish()
