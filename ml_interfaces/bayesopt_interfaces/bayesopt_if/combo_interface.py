#!/usr/bin/env python
# -*- coding: utf-8 -*-

from __future__ import print_function
import os
import sys
import numpy as np
import pandas as pd
import pickle
import matplotlib.pyplot as plt


PYTHON_VERSION = sys.version_info.major
if PYTHON_VERSION == 2:
    import combo
    print('PYTHON_VERSION is 2, so COMBO is imported')
elif PYTHON_VERSION == 3:
    print('PYTHON_VERSION is 3, so COMBO is not imported')


class ComboInterface():
    def __init__(self, bayesopt, candidates_path='candidates.csv', policy_load_dir='.', policy_save_dir='.', use_saved_policy=False, visualize=True, search_score='PI'):
        self.data = None
        self.X = None
        self.actions = None
        self.values = None
        self.param_num = 0
        self.policy = None
        self.next_action = None
        self.candidates_path = candidates_path
        self.policy_load_dir = policy_load_dir
        self.policy_save_dir = policy_save_dir
        self.use_saved_policy = use_saved_policy
        self.visualize = visualize
        self.search_mode = 0  # 0: random search, 1: bayes
        self.search_score = search_score  # score: PI, EI, TS
        self.is_write_completed = False
        self.is_search_completed = False
        self.chosed_actions = None
        self.remaining_actions = None
        self.bayesopt = bayesopt


    def start_bayesopt(self):
        # load csv
        self.load_dataset()

        # generate policy
        if self.use_saved_policy:
            # start from existing policy(npz and dump)
            self.load_policy()
        else:
            # start experiment at first
            self.generate_policy()

        # for visualization
        if self.visualize:
            self.fig, self.ax = plt.subplots(2, 1, tight_layout=True)  # avoid overlap of labels

            # visualize
            self.visualize()


    def load_dataset(self):
        self.data = pd.read_csv(self.candidates_path)
        self.X = self.data.iloc[:,:-1].to_numpy()
        self.actions = list(self.data[self.data["y"] < np.inf].index)
        self.values = self.data.iloc[self.actions,-1].to_numpy()
        self.param_num = len(self.X[0])

        # calculation values
        self.mean = np.zeros(len(self.X))
        self.std = np.zeros(len(self.X))
        self.score_PI = np.zeros(len(self.X))  # acquisition function
        self.score_EI = np.zeros(len(self.X))
        self.score_TS = np.zeros(len(self.X))


    # COMBO works as a MAXIMIZER in defalut
    ## To avoid confusing,
    ## - if you want to use COMBO as MINIMIZER, objective function should be changed to maximize the function with evaluation values.
    ## - if you already have dataset for MINIMIZER, generating another dataset is recommended.
    def generate_policy(self):
        self.policy = self.bayesopt.search.discrete.policy(self.X)
        self.update_remaining_actions()  # remaining_actions should be updated here
        print('COMBO instance is generated')
        print('search_score: ', self.search_score)


    def load_policy(self):
        print('Loading policy from ', self.policy_load_dir, '\n')
        self.policy = self.bayesopt.search.discrete.policy(self.X)
        self.policy.load(file_history = os.path.join(self.policy_load_dir, 'history.npz'),
                         file_training = os.path.join(self.policy_load_dir, 'training.npz'),
                         file_predictor = os.path.join(self.policy_load_dir, 'predictor.dump'))

        # update values with initial data
        self.update_bayesopt()

        print('Policy has been loaded\n')


    def inform_search_mode(self):
        if self.search_mode == 0:
            print('Current search mode: RANDOM')
        elif self.search_mode == 1:
            print('Current search mode: BAYES_OPT')


    # this method will be executed to get next param
    def search_next_param(self):
        self.is_search_completed = False
        self.is_write_completed = False  # write flag off before search

        # check actions
        if self.remaining_actions.size == 0:
            print('\033[31m' + 'All actions have been evaluated. Save and Exit\n' + '\033[0m')
            self.save_data(self.policy_save_dir)
            exit()

        # search
        if self.search_mode == 0:
            self.search_next_param_random()
        elif self.search_mode == 1:
            self.search_next_param_bayes()

        self.is_search_completed = True


    def search_next_param_bayes(self, **kwargs):
        print('Search next param by BAYES_OPT (score:', self.search_score, ')')
        self.next_action = self.policy.bayes_search(score=self.search_score, **kwargs)
        print(' Next index: {}'.format(self.next_action))
        print(' Next param: {}'.format(self.X[self.next_action][0]))


    def search_next_param_random(self, **kwargs):
        print('Search next param by RANDOM search')
        self.next_action = self.policy.random_search(max_num_probes=1, **kwargs)
        print(' Next index: {}'.format(self.next_action))
        print(' Next param: {}'.format(self.X[self.next_action][0]))


    def write_result(self, value):
        self.is_search_completed = False  # search flag off before write
        self.is_write_completed = False

        self.policy.write(self.next_action, value)
        print('\nShow history:')
        self.bayesopt.search.utility.show_search_results(self.policy.history, 10)

        self.is_write_completed = True

        # update combo after write
        self.update_bayesopt()


    def save_data(self, policy_save_dir='.'):
        self.policy_save_dir = policy_save_dir

        # generate save dir
        if not os.path.isdir(self.policy_save_dir):
            os.mkdir(self.policy_save_dir)

        # update before save
        print('Update bayesopt before save')
        self.update_bayesopt()

        # save
        self.save_policy(self.policy_save_dir)
        self.save_candidates_and_actions(self.policy_save_dir)  # csv
        self.save_candidates_with_scores(self.policy_save_dir)  # csv
        if self.visualize:
            self.save_plt(self.policy_save_dir)


    def save_policy(self, policy_save_dir='.'):
        file_predictor = os.path.join(policy_save_dir, 'predictor.dump')
        file_training = os.path.join(policy_save_dir, 'training.npz')
        file_history = os.path.join(policy_save_dir, 'history.npz')
        with open(file_predictor, 'wb+') as f:
            pickle.dump(self.policy.predictor, f)
        self.policy.training.save(file_training)
        self.policy.history.save(file_history)


    def save_candidates_and_actions(self, policy_save_dir='.'):
        # candidates
        candidates_each_exp_path = os.path.join(policy_save_dir, 'candidates.csv')
        for i,a in enumerate(self.chosed_actions):
            self.data["y"].iloc[a] = self.policy.training.t[i]
            self.data.to_csv(self.candidates_path, index=False)  # for saving latest version
            self.data.to_csv(candidates_each_exp_path, index=False)  # for archiving and loading candidate in resume exp

        # chosed actions
        chosed_actions_path = os.path.join(os.path.dirname(self.candidates_path), 'chosed_actions.csv')
        chosed_actions_each_exp_path = os.path.join(policy_save_dir, 'chosed_actions.csv')
        df_chosed_actions = pd.Series(self.chosed_actions)
        df_chosed_actions.to_csv(chosed_actions_path, index=False, header=False)  # latest version
        df_chosed_actions.to_csv(chosed_actions_each_exp_path, index=False, header=False)  # archive


    def save_candidates_with_scores(self, policy_save_dir='.'):
        # append scores to candidate data and save
        data_with_scores = self.data.copy()
        data_with_scores['mean'] = self.mean
        data_with_scores['std'] = self.std
        data_with_scores['mean-std'] = self.mean - self.std
        data_with_scores['mean+std'] = self.mean + self.std
        data_with_scores['score_EI'] = self.score_EI
        data_with_scores['score_PI'] = self.score_PI
        data_with_scores['score_TS'] = self.score_TS
        candidates_with_scores_path = os.path.join(policy_save_dir, 'candidates_with_scores.csv')
        data_with_scores.to_csv(candidates_with_scores_path, index=False)


    def save_plt(self, policy_save_dir='.'):
        # update before save
        self.visualize()
        # save plot
        plt_save_path_png = os.path.join(policy_save_dir, 'plt.png')
        plt_save_path_pdf = os.path.join(policy_save_dir, 'plt.pdf')
        plt.savefig(plt_save_path_png, format='png', dpi=300)
        plt.savefig(plt_save_path_pdf, format='pdf', dpi=300)


    def delete_policy(self):
        del self.policy


    def visualize(self):
        # combo specific
        if self.policy.training.t is None:
            self.policy.training.t = np.array([])

        # visualize evaluation value
        self.ax[0].cla()  # reset plotted graph
        self.ax[0].plot(self.policy.training.t, ".", label="sample")
        self.ax[0].plot(pd.DataFrame(self.policy.training.t).cummax(), label="cummax")
        self.ax[0].set_title('Cummax')
        self.ax[0].set_xlabel("Experiment id")
        self.ax[0].set_ylabel("Evaluation value")
        self.ax[0].set_xlim(0,)
        self.ax[0].set_xticks(np.arange(0, len(self.policy.training.t), 1))
        self.ax[0].legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

        # visualize score
        self.ax[1].cla()
        self.ax[1].plot(self.score_EI, label='EI')  # score is ordered same as candidates?
        self.ax[1].plot(self.score_PI, label='PI')
        self.ax[1].plot(self.score_TS, label='TS')
        self.ax[1].set_title("Acquisition function")
        self.ax[1].set_xlabel("Candidate (action) id")
        self.ax[1].set_ylabel("Function value")
        self.ax[1].set_xlim(0,)
        self.ax[1].legend(bbox_to_anchor=(1.05, 1), loc='upper left', borderaxespad=0)

        plt.pause(0.1)  # use pause() for continuous plot


    # at least 1 evaluation data is necessary to update_bayesopt
    def update_bayesopt(self):
        print('Updating bayesopt:')
        self.policy.bayes_search(max_num_probes=0, num_rand_basis=0)  # need to avoid warning
        self.update_mean()
        self.update_std()
        self.update_score()
        self.update_chosed_actions()
        self.update_remaining_actions()
        print('\n')


    def update_score(self):
        print('Updating score')
        self.policy.predictor.prepare(self.policy.training)  # prepare() is needed before get_post_score
        self.score_EI = self.bayesopt.search.score.EI(predictor=self.policy.predictor, training=self.policy.training, test=self)
        self.score_PI = self.bayesopt.search.score.PI(predictor=self.policy.predictor, training=self.policy.training, test=self)
        self.score_TS = self.bayesopt.search.score.TS(predictor=self.policy.predictor, training=self.policy.training, test=self)[0]
        print('EI:\n', self.score_EI)
        print('PI:\n', self.score_PI)
        print('TS:\n', self.score_TS)


    def update_mean(self):
        print('Updating mean')
        self.policy.predictor.prepare(self.policy.training)  # prepare() is needed before get_post_fmean
        self.mean = self.policy.predictor.model.get_post_fmean(self.policy.training.X, self.X)
        print('mean:\n', self.mean)


    def update_std(self):
        print('Updating std')
        self.policy.predictor.prepare(self.policy.training)  # prepare() is needed before get_post_fcov
        var = self.policy.predictor.model.get_post_fcov(self.policy.training.X, self.X)
        self.std = np.sqrt(var)
        print('std:\n', self.std)


    def update_chosed_actions(self):
        self.chosed_actions = self.policy.history.chosed_actions[:self.policy.history.total_num_search]
        print('chosed_actions:\n', self.chosed_actions)


    def update_remaining_actions(self):
        self.remaining_actions = self.policy.actions
        print('remaining_actions:\n', self.remaining_actions)


    def get_next_index(self):
        if self.next_action is not None:
            next_index = self.next_action[0]
        else:
            next_index = -1

        return next_index


    def get_next_param(self):
        if self.next_action is not None:
            next_param = self.X[self.next_action[0]]
        else:
            next_param = [0] * self.param_num

        return next_param
