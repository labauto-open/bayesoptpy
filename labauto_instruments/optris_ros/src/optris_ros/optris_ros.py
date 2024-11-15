#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import Float64, Int64
#from optris_ros.optris_interface import OptrisInterface
from optrispy.optris_interface import OptrisInterface


class OptrisROS():
    def __init__(self):
        # variables
        self.temp = 0
        self.focusmotor_pos = 0

        # publisher
        self.pub_temp = rospy.Publisher('temperature', Float64, queue_size=10)
        self.pub_focusmotor_pos = rospy.Publisher('focusmotor_pos', Int64, queue_size=10)

        # instance
        self.optris = OptrisInterface()
        self.optris.connect_optris()  # open serial port


    def update_temp(self):
        self.temp = self.optris.get_temperature()


    def update_focusmotor_pos(self):
        self.focusmotor_pos = self.optris.get_focusmotor_pos()


    '''
    def update_xxx(self):
        xxx
    '''


    def update_data(self):
        self.update_temp()
        self.update_focusmotor_pos()
        # self.update_xxx


    def publish_temp(self):
        self.update_temp()
        self.pub_temp.publish(self.temp)


    def publish_data(self):
        self.update_data()
        self.pub_temp.publish(self.temp)
        self.pub_focusmotor_pos.publish(self.focusmotor_pos)
        # self.pub_xxx
