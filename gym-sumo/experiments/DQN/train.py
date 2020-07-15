"""
Train an agent using stable_baselines for sumo experiment.

todo add options to determine whether using sumo-gui
"""

import gym
import gym_sumo  # this line is necessary

# from stable_baselines.common.policies import MlpPolicy
from stable_baselines.deepq import MlpPolicy

from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

from stable_baselines import DQN



# register self-defined sumo env
env = gym.make('sumo-v0')

# print('d')


# route selection
route_selection = ['left', 'mid', 'right']

route = route_selection[0]

algorithm_name = 'dqn'

log_path = './outputs/log/'+'/'+route+'/'

dict_path = './outputs/model_dict/'+algorithm_name+'_'+route

# set route of training
env.route = route

# DQN model
model = DQN(MlpPolicy, env, buffer_size=50000, verbose=1, tensorboard_log=log_path, seed=1)

model.learn(total_timesteps=10000)

model.save(dict_path)


# todo train with other algorithms, i.e. ppo

# Optional: PPO2 requires a vectorized environment to run
# the env is now wrapped automatically when passing it to the constructor
# env = DummyVecEnv([lambda: env])

# model = PPO2(MlpPolicy, env, verbose=1)




