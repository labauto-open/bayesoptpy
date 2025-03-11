#!/usr/bin/env python
# -*- coding: utf-8 -*-

import csv
import filecmp
import os
import random
import sys

import combo
from bayesopt_if.combo_interface import ComboInterface

file_dir = os.path.dirname(__file__)
data_dir = os.path.join(file_dir, 'data')


def get_data_from_result_list(index):
    results_path = os.path.join(data_dir, 'results.csv')
    with open(results_path) as f:
        reader = csv.reader(f)
        l = [row for row in reader]

    return float(l[index+1][0])


def test_combo_interface():
    # dir setting
    candidates_path  = os.path.join(data_dir, 'candidates.csv')
    correct_candidates_path = os.path.join(data_dir, 'candidates-correct.csv')
    policy_load_dir = os.path.join(data_dir, 'load')
    policy_save_dir = os.path.join(data_dir, 'result')

    # Generate combo
    ci = ComboInterface(combo, candidates_path, policy_load_dir, policy_save_dir, use_saved_policy=False, visualize=False)
    ci.start_bayesopt()

    # Write combo operation below
    ## Following procedure is important for correct generation of result data.
    ## search -> write(and update -> visualize)
    for i in range(3):
        ci.search_next_param_random()
        result = get_data_from_result_list(ci.get_next_index())  # get result/evaluation data from experiment. (Result list is used for this test program).
        ci.write_result(result)  # write result
    for i in range(17):
        ci.search_next_param_bayes()
        result = get_data_from_result_list(ci.get_next_index())
        ci.write_result(result)

    #raw_input('Press enter to save and finish\n')
    ci.save_data(policy_save_dir)

    assert filecmp.cmp(candidates_path, correct_candidates_path)
