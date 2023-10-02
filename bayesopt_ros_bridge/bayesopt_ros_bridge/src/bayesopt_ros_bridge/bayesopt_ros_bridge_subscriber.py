#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import rospy
import roslib.packages
from std_msgs.msg import *


class BayesoptROSBridgeSubscriber(object):
    def __init__(self, config, bayesopt, data_save_dir):
        self.bayesopt = bayesopt

        # common subscribers
        rospy.Subscriber('/bayesopt_ros_bridge/start_param_search', Bool, self.param_search_cb, queue_size=1)
        rospy.Subscriber('/bayesopt_ros_bridge/save_data', Bool, self.save_data_cb, queue_size=1)

        # experiment specific subscribers
        cfg = config['result']
        rospy.Subscriber(cfg['topic_name'], eval(cfg['topic_type']), self.write_result_cb, queue_size=1)

        cfg = config['data_save_dir']
        rospy.Subscriber(cfg['topic_name'], eval(cfg['topic_type']), self.data_save_dir_cb, queue_size=1)
        self.data_save_dir = data_save_dir  # initial path is loaded from arg


    def write_result_cb(self, msg):
        # write result and update
        self.bayesopt.write_result(msg.data)


    def param_search_cb(self, msg):
        # search next param
        self.bayesopt.search_next_param()


    def data_save_dir_cb(self, msg):
        # data_save_dir will be updated by topic
        self.data_save_dir = msg.data


    def save_data_cb(self, msg):
        # save
        rospy.loginfo('Saving data in %s', self.data_save_dir)
        self.bayesopt.save_data(self.data_save_dir)
