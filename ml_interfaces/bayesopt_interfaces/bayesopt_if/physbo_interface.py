#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import itertools as iter
import os
import numpy as np
import pandas as pd
import physbo
import matplotlib.pyplot as plt
import yaml
from .combo_interface import ComboInterface


class PhysboInterface(ComboInterface):
    def __init__(self, candidates_path='candidates.csv', policy_load_dir='.', policy_save_dir='.', use_saved_policy=False, visualize=True, search_score='PI'):
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
                         self.visualize,
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
    def save_candidates_with_scores(self, policy_save_dir='.'):
        selected_order = np.full(len(self.X), np.nan)
        for i,a in enumerate(self.chosen_actions):
            selected_order[a] = i
        print('selected_order:', selected_order)

        # append scores to candidate data and save
        data_with_scores = self.data.copy()
        data_with_scores['selected_order'] = selected_order
        data_with_scores['mean'] = self.mean
        data_with_scores['std'] = self.std
        data_with_scores['mean-std'] = self.mean_minus_std
        data_with_scores['mean+std'] = self.mean_plus_std
        data_with_scores['score_EI'] = self.score_EI
        data_with_scores['score_PI'] = self.score_PI
        data_with_scores['score_TS'] = self.score_TS

        candidates_with_scores_path = os.path.join(policy_save_dir, 'candidates_with_scores.csv')
        data_with_scores.to_csv(candidates_with_scores_path, index=False)


    # override
    def update_score(self):
        print('Updating score')
        if self.search_score=='EI':
            self.score_EI = self.policy.get_score(mode="EI", xs=self.X)
            print('EI:\n', self.score_EI)
        elif self.search_score=='PI':
            self.score_PI = self.policy.get_score(mode="PI", xs=self.X)
            print('PI:\n', self.score_PI)
        elif self.search_score=='TS':
            self.score_TS = self.policy.get_score(mode="TS", xs=self.X)
            print('TS:\n', self.score_TS)
        else:
            print('unknown function')


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


    def generate_candidates(self, candidates_config_path='../../config/candidates_config.yaml'):
        with open(candidates_config_path, 'r') as yml:
            config = yaml.load(yml, Loader=yaml.Loader)
            candidates_cfg = config['config']['param_candidates']

        # set values
        candidates_label = candidates_cfg['label']
        candidates_label_str = ','.join(candidates_label)  # label needs to be string with NO SPACE between parameters.
        param_dict = candidates_cfg['param'][0]
        param_num = len(candidates_cfg['param'][0])

        # param list
        param_value_list = []
        for v in param_dict.values():
            param_value_list.append(v)

        # index list for loop
        # 'ex: i1, i2, i3'
        i_list = []
        for i in range(param_num):
            i_list.append('i' + str(i))

        # output csv format
        # - NO SPACE between parameters
        # - 'ex: %d,%d,%d,\n'
        write_format = ''
        for i in range(param_num):
            write_format += '%d,'
        write_format += '\n'

        try:
            print('Generating %s from %s' % (self.candidates_path, candidates_config_path))
            write_mode='x'

            # check dir
            candidates_dir = os.path.dirname(self.candidates_path)
            if not os.path.exists(candidates_dir):
                print('candidates_dir is not exists. Do you make it [y/N]?')
                val = input()
                if val == 'y' or val == 'Y':
                    os.makedirs(candidates_dir)
                    print('%s is generated' % (candidates_dir))
                else:
                    print('exit')
                    exit()

            # check candidates
            if os.path.exists(self.candidates_path):
                print('%s already exists. Do you overwrite [y/N]?' % (self.candidates_path))
                val = input()
                if val == 'y' or val == 'Y':
                    write_mode='w'
                else:
                    print('exit')
                    exit()

            # generate candidates
            with open(self.candidates_path, mode=write_mode) as f:
                f.write(candidates_label_str)
                f.write('\n')
                for i_list in iter.product(*param_value_list):  # unpack
                    f.write(write_format % i_list)
                print('%s is generated' % (self.candidates_path))

        except Exception as e:
            print('Error', e)
