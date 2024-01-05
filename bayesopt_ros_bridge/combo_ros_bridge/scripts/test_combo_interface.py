#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import random
import combo
from combo_ros_bridge.combo_interface import ComboInterface


def main():
    # dir setting
    candidates_path  = '../data/test_candidates.csv'
    policy_load_dir = '../data/test_result'
    policy_save_dir = '../data/test_result'

    # Generate combo
    ci = ComboInterface(combo, candidates_path, policy_load_dir, policy_save_dir, use_saved_policy=False)
    ci.start_bayesopt()

    # Write combo operation below
    ## Following procedure is important for correct generation of result data.
    ## search -> write(and update -> visualize)
    for i in range(1):
        ci.search_next_param_random()
        ci.write_result(random.random())  # write result
    for i in range(1):
        ci.search_next_param_bayes()
        ci.write_result(random.random())  # write result

    raw_input('Press enter to save and finish\n')

    ci.save_data(policy_save_dir)


if __name__ == '__main__':
    main()
