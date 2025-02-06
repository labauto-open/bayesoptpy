#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import *


class BayesoptROSBridgePublisher(object):
    def __init__(self, config, bayesopt):
        self.bayesopt = bayesopt

        self.next_index = -1
        self.next_param = []

        self.msg_type_next_param = None
        self.msg_next_index = None
        self.msg_next_param = None
        self.msg_complete_write_result = Bool(data=False)
        self.msg_complete_param_search = Bool(data=False)

        # common publishers
        self.pub_next_index = rospy.Publisher('/bayesopt_ros_bridge/next_index', Int16, queue_size=1)
        self.pub_complete_write_result = rospy.Publisher('/bayesopt_ros_bridge/complete_write_result', Bool, queue_size=1)
        self.pub_complete_param_search = rospy.Publisher('/bayesopt_ros_bridge/complete_param_search', Bool, queue_size=1)

        # experiment specific publishers
        cfg = config['next_param']
        self.pub_next_param = rospy.Publisher(cfg['topic_name'], eval(cfg['topic_type']), queue_size=1)
        self.msg_type_next_param = eval(cfg['topic_type'])


    def publish(self):
        # update next index and param
        self.next_index = self.bayesopt.get_next_index()
        self.next_param = self.bayesopt.get_next_param()

        # generate ROS msg
        self.msg_next_index = Int16(data=self.next_index)
        self.msg_next_param = self.msg_type_next_param(data=self.next_param)
        self.msg_complete_write_result = Bool(data=self.bayesopt.is_write_completed)
        self.msg_complete_param_search = Bool(data=self.bayesopt.is_search_completed)

        self.pub_next_index.publish(self.msg_next_index)
        self.pub_next_param.publish(self.msg_next_param)
        self.pub_complete_write_result.publish(self.msg_complete_write_result)
        self.pub_complete_param_search.publish(self.msg_complete_param_search)
