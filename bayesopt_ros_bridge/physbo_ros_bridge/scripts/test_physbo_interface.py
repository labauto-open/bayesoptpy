#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import random
from physbo_ros_bridge.physbo_interface import PhysboInterface


def main():
    # dir setting
    candidates_path  = '../data/test_candidates.csv'
    policy_load_dir = '../data/test_result'
    policy_save_dir = '../data/test_result'

    # Generate physbo
    pi = PhysboInterface(candidates_path, policy_load_dir, policy_save_dir, use_saved_policy=False)
    pi.start_bayesopt()

    # Write physbo operation below
    ## Following procedure is important for correct generation of result data.
    ## search -> write (and update -> visualize)
    for i in range(1):
        pi.search_next_param_random()
        pi.write_result(random.random())  # write result
    for i in range(1):
        pi.search_next_param_bayes()
        pi.write_result(random.random())  # write result

    input('Press enter to save and finish\n')

    pi.save_data(policy_save_dir)


if __name__ == '__main__':
    main()
