#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os
import numpy as np
import pandas as pd
import physbo
import matplotlib.pyplot as plt
from combo_ros_bridge.combo_interface import ComboInterface


class PhysboInterface(ComboInterface):
    def __init__(self, candidates_path='candidates.csv', policy_load_dir='.', policy_save_dir='.', use_saved_policy=False, search_score='PI'):
        self.candidates_path = candidates_path
        self.policy_load_dir = policy_load_dir
        self.policy_save_dir = policy_save_dir
        self.use_saved_policy = use_saved_policy
        self.search_score = search_score

        self.chosen_actions = None

        super().__init__(physbo,
                         self.candidates_path,
                         self.policy_load_dir,
                         self.policy_save_dir,
                         self.use_saved_policy,
                         self.search_score)  # python3 description


    # PHYSBO works as a MAXIMIZER in defalut
    ## https://issp-center-dev.github.io/PHYSBO/manual/develop/ja/notebook/tutorial_basic.html
    ## To avoid confusing,
    ## - if you want to use PHYSBO as MINIMIZER, objective function should be changed to maximize the function with evaluation values.
    ## - if you already have dataset for MINIMIZER, generating another dataset is recommended.

    # override
    def generate_policy(self):
        self.policy = self.bayesopt.search.discrete.policy(self.X, initial_data=(self.actions, self.values))
        self.update_remaining_actions()  # remaining_actions should be updated here
        print('PHYSBO instance is generated')
        print('search_score: ', self.search_score)


    # override
    def save_policy(self, policy_save_dir='.'):
        self.policy.save(file_history = os.path.join(policy_save_dir, 'history.npz'),
                         file_training = os.path.join(policy_save_dir, 'training.npz'),
                         file_predictor = os.path.join(policy_save_dir, 'predictor.dump'))


    # override
    def save_candidates_and_actions(self, policy_save_dir='.'):
        # candidates
        candidates_each_exp_path = os.path.join(policy_save_dir, 'candidates.csv')
        for i,a in enumerate(self.chosen_actions):
            self.data["y"].iloc[a] = self.policy.training.t[i]
            self.data.to_csv(self.candidates_path, index=False)  # for saving latest version
            self.data.to_csv(candidates_each_exp_path, index=False)  # for archiving and loading candidate in resume exp

        # chosen actions
        chosen_actions_path = os.path.join(os.path.dirname(self.candidates_path), 'chosen_actions.csv')
        chosen_actions_each_exp_path = os.path.join(policy_save_dir, 'chosen_actions.csv')
        df_chosen_actions = pd.Series(self.chosen_actions)
        df_chosen_actions.to_csv(chosen_actions_path, index=False, header=False)  # latest version
        df_chosen_actions.to_csv(chosen_actions_each_exp_path, index=False, header=False)  # archive


    # override
    def update_score(self):
        print('Updating score')
        self.score_EI = self.policy.get_score(mode="EI", xs=self.X)
        self.score_PI = self.policy.get_score(mode="PI", xs=self.X)
        self.score_TS = self.policy.get_score(mode="TS", xs=self.X)
        print('EI:\n', self.score_EI)
        print('PI:\n', self.score_PI)
        print('TS:\n', self.score_TS)


    # override
    def update_mean(self):
        print('Updating mean')
        self.mean = self.policy.get_post_fmean(self.X)
        print('mean:\n', self.mean)


    # override
    def update_std(self):
        print('Updating std')
        var = self.policy.get_post_fcov(self.X)
        self.std = np.sqrt(var)
        print('std:\n', self.std)


    # override
    # term was changed. chosed_action to chosen_action
    def update_chosed_actions(self):
        self.chosen_actions = self.policy.history.chosen_actions[:self.policy.history.total_num_search]
        print('chosen_actions:\n', self.chosen_actions)
