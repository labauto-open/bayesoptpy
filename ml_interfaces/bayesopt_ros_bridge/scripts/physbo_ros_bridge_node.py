#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import rospy
from bayesopt_if.physbo_interface import PhysboInterface
from bayesopt_ros_bridge.physbo_ros_bridge import PhysboROSBridge


def main():
    rospy.init_node('physbo_ros_bridge_node', anonymous=True)

    # Get the parameters
    try:
        pub_rate = rospy.get_param('~pub_rate', 1)
    except:
        rospy.logfatal('Failed to get "pub_rate" parameter')
        exit(1)

    try:
        resume_data_dir = rospy.get_param('~resume_data_dir')
    except:
        rospy.logfatal('Failed to get "resume_data_dir" parameter')
        exit(1)

    try:
        candidates_path = rospy.get_param('~candidates_path')
    except:
        rospy.logfatal('Failed to get "candidates_path" parameter')
        exit(1)

    try:
        policy_save_dir = rospy.get_param('~data_save_dir')
    except:
        rospy.logfatal('Failed to get "data_save_dir" parameter')
        exit(1)

    try:
        config = rospy.get_param('~config')
    except:
        rospy.logfatal('Failed to get "config" parameter')
        exit(1)

    # dir setting
    if resume_data_dir is False:
        policy_load_dir = ""
        use_saved_policy = False
    else:
        policy_load_dir = resume_data_dir
        use_saved_policy = True

    # PhysboInterface setting
    search_score = config['acquisition_function']

    # Generate physbo and bridge
    pi = PhysboInterface(candidates_path, policy_load_dir, policy_save_dir, use_saved_policy, search_score)
    bridge = PhysboROSBridge(pi, config, pub_rate, policy_save_dir)

    # Start bayesopt and bridge
    pi.start_bayesopt()
    bridge.run()
    rospy.spin()

    pi.save_data(policy_save_dir)
    pi.delete_policy()


if __name__ == '__main__':
    main()
