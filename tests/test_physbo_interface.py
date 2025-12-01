#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import csv
import filecmp
import os
import sys

from bayesoptpy.physbo_interface import PhysboInterface

file_dir = os.path.dirname(__file__)
data_dir = os.path.join(file_dir, 'data')


def get_data_from_result_list(index):
    results_path = os.path.join(data_dir, 'results.csv')
    with open(results_path) as f:
        reader = csv.reader(f)
        l = [row for row in reader]

    return float(l[index+1][0])


def test_physbo_interface():
    # dir setting
    candidates_path  = os.path.join(data_dir, 'candidates.csv')
    correct_candidates_path = os.path.join(data_dir, 'candidates-correct.csv')
    policy_load_dir = os.path.join(data_dir, 'load')
    policy_save_dir = os.path.join(data_dir, 'result')

    # Generate physbo
    pi = PhysboInterface(candidates_path, policy_load_dir, policy_save_dir, use_saved_policy=False, search_score='EI')
    pi.start_bayesopt()

    # Procedure of search and register data to PHYSBO
    # - Following procedure is important for correct generation of result data.
    # - search -> write (and update -> visualize)

    # Search with Random search for initial step
    for i in range(3):
        pi.search_next_param_random()
        result = get_data_from_result_list(pi.get_next_index())  # get result/evaluation data from experiment. (Result list is used for this test program).
        pi.write_result(result)  # write result to PHYSBO
    # Search with Bayesian optimization
    for i in range(17):
        pi.search_next_param_bayes()
        result = get_data_from_result_list(pi.get_next_index())
        pi.write_result(result)

    #input('Press enter to save and finish\n')
    pi.save_data(policy_save_dir)

    assert filecmp.cmp(candidates_path, correct_candidates_path)
