#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
from optris_ros.optris_ros import OptrisROS


def main():
    rospy.init_node('xi80_ros_node', anonymous=True)

    xi80_ros = OptrisROS()

    # Run publisher
    while not rospy.is_shutdown():
        xi80_ros.publish_temp()


if __name__ ==  "__main__":
    main()
