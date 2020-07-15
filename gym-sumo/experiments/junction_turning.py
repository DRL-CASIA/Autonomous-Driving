
from env.BasicEnv import SUMOEnv
from algorithms.DQNAgent import DQNAgent

import numpy as np
import math
import torch
from collections import namedtuple


Transition = namedtuple('Transition', ['state', 'action', 'reward', 'next_state'])

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

cfg_file = '../config/test.sumocfg'


class JunctionSim:

    episode = 1000
    state_dim = 30
    action_dim = 4

    def __init__(self, config, use_gui=True, route='right'):
        """"""
        # assign env
        self.env = SUMOEnv(config, use_gui)
        # assign agent
        self.agent = DQNAgent(self.state_dim, self.action_dim, route)

        # select route
        self.env.route = route
        self.agent.get_route_name(route)

        # get episode number for agent
        self.agent.get_episode(self.episode)

        # parameters for RL training
        self.state = None
        self.next_state = None
        self.action = None
        self.reward = None
        self.done = False

    # def main_loop(self):
    #     """
    #     Run each episode.
    #     Update after the episode is finished.
    #     :return:
    #     """
    #     pass

    def run(self, train_flag='train'):
        """
        Run the training proess or evaluation.
        Use train flag to control run mode.
        :param train_flag:
        train: train without loading previous dict
        tune: continue training with previous dict
        eval: evaluating previous trained weights.
        """

        if train_flag is not 'train':
            # load previous Net dict
            self.agent.load_net()

        if train_flag == 'eval':
            self.episode = 100

        # run episodes
        for ep in range(self.episode):

            print('*' * 20)
            print('-' * 20)
            print('*' * 20)
            print('episode: ', ep)

            # set episode number to agent
            self.agent.get_episode_index(ep)
            self.agent.change_rate(ep)

            # reset ego vehicle and get initial state
            state = self.env.reset()

            # step number of each episode
            step_number = 0

            # episode main loop
            while True:

                # print('==================================================')
                # print('step: ', step_number)

                # update transition
                if self.state is None:
                    self.state = state
                else:
                    self.next_state = state
                    transition = Transition(self.state, self.action, self.reward, self.next_state)  # store transition
                    self.agent.store_transition(transition)
                    self.state = self.next_state  # update transition

                # todo check correct order of this part
                self.action = self.agent.select_action(state)
                # print('action(target speed): ', self.env.action_space[self.action], ' m/s')

                # step the env
                state, self.reward, self.done, aux = self.env.step(self.action)

                print('state dim: ', len(state))

                # ==================================================
                # test collision
                # state, self.reward, self.done, aux = self.env.step(3)
                # ==================================================

                step_number += 1

                # end the simulation
                if self.done:
                    break

            print('episode: ', ep, 'episode reward: ', self.reward)
            print('episode time step: ', step_number, 'episode time: ', self.env.episode_time)

            if train_flag == 'train' or train_flag == 'tune':
                # update NN parameters after each episode
                self.agent.update()
                #
                self.agent.save_net()

            # save net each N episodes
            # epi_interval = 20
            # if (ep + 1) % epi_interval == 0:
            #     self.agent.save_net()

        print('=' * 20)
        print('*' * 20)
        print('=' * 20)

        print("Experiment is done.")
        print('success counts: ', self.env.result_count['success'], '/', self.episode)
        print('collision counts: ', self.env.result_count['collision'], '/', self.episode)
        print('time exceed counts: ', self.env.result_count['time_exceed'], '/', self.episode)


if __name__ == '__main__':

    """
    train: train without loading previous dict
    tune: continue training with previous dict
    eval: evaluating previous trained weights.
    """

    train_flag = 'eval'

    # available route
    # ['left', 'mid', 'right']
    route = 'right'

    use_gui = False
    if train_flag == 'eval':
        use_gui = True

    sim = JunctionSim(cfg_file, use_gui=use_gui, route=route)

    sim.run(train_flag)

