#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import *
from bayesopt_ros_bridge.bayesopt_ros_bridge import BayesoptROSBridge


class ComboROSBridge(BayesoptROSBridge):
    def __init__(self, bayesopt_interface, config, pub_rate, policy_save_dir):
        self.bayesopt_interface = bayesopt_interface
        self.config = config
        self.pub_rate = pub_rate
        self.policy_save_dir = policy_save_dir
        super(ComboROSBridge, self).__init__(self.bayesopt_interface, self.config, self.pub_rate, self.policy_save_dir)  # python2 description

        # Subscriber
        rospy.Subscriber('/bayesopt_ros_bridge/change_search_mode', Int8, self.change_search_mode_cb, queue_size=1)

        # Publisher
        self.msg_search_mode = None
        self.pub_search_mode = rospy.Publisher('/bayesopt_ros_bridge/search_mode', Int8, queue_size=1)


    # callback
    def change_search_mode_cb(self, msg):
        # flag off at this init timing
        ## procedure:
        ## change search mode -> search -> write
        self.bayesopt_interface.is_search_completed = False
        self.bayesopt_interface.is_write_completed = False

        # change mode
        if (msg.data == 0 or msg.data == 1):
            self.bayesopt_interface.search_mode = msg.data

        else:
            rospy.logwarn('search_mode {} is not implemented yet'.format(msg.data))

        rospy.loginfo('Changing search mode')
        self.bayesopt_interface.inform_search_mode()


    # publish
    def publish_search_mode(self):
        # combo specific
        self.msg_search_mode = Int8(data=self.bayesopt_interface.search_mode)
        self.pub_search_mode.publish(self.msg_search_mode)


    # override
    def update(self, event):
        # check search completed
        self.pub.publish()
        self.publish_search_mode()
