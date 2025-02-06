#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import *
from bayesopt_ros_bridge.combo_ros_bridge import ComboROSBridge


class PhysboROSBridge(ComboROSBridge):
    def __init__(self, physbo_interface, config, pub_rate, policy_save_dir):
        self.physbo_interface = physbo_interface
        self.config = config
        self.pub_rate = pub_rate
        self.policy_save_dir = policy_save_dir
        super(PhysboROSBridge, self).__init__(self.physbo_interface, self.config, self.pub_rate, self.policy_save_dir)
