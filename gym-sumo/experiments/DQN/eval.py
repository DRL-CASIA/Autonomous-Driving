"""
Try eval a model.

"""

import gym
import gym_sumo  # this line is necessary

# from stable_baselines.common.policies import MlpPolicy
from stable_baselines.deepq import MlpPolicy

from stable_baselines.common.vec_env import DummyVecEnv
from stable_baselines import PPO2

from stable_baselines import DQN

from statistics import mean


# training process
# env = gym.make('CartPole-v1')
env = gym.make('sumo-v0')

show_route = env.route

# print('d')

# route selection
route_selection = ['left', 'mid', 'right']

route = route_selection[0]

# get model dict
algorithm_name = 'dqn'

dict_path = './outputs/model_dict/'+algorithm_name+'_'+route

# set route of training
env.route = route

show_route = env.route

print('d')

# model has to be same
model = DQN.load(dict_path)

print('d')

# visualize
obs = env.reset()

step = 0  # step in one episode
epi_num = 0  # episode number
success = 0
failure = 0
collision = 0
time_exceed = 0
episode_time_record = []

run_num = 100

while epi_num < run_num:
    # print('step_num: ', step)
    action, _states = model.predict(obs)
    obs, rewards, dones, info = env.step(action)
    step += 1

    if dones:
        print('Episode No.: ', epi_num, 'finish. Result: ', info['exp_state'])
        print('total step of this episode: ', step)
        # record result
        if info['exp_state'] == 'collision':
            collision += 1
            failure += 1
        elif info['exp_state'] == 'time_exceed':
            time_exceed += 1
            failure += 1
        else:
            print('Episode ', epi_num, info['exp_state'])
            # get episode time
            episode_time_record.append(env.episode_time)

            success += 1

        step = 0
        epi_num += 1
        obs = env.reset()


print('-*'*15, ' result ', '-*'*15)
print('success: ', success, '/', run_num)
print('collision: ', collision, '/', run_num)
print('time_exceed: ', time_exceed, '/', run_num)
print('length of episode_time_record: ', len(episode_time_record))
print('average time: ', mean(episode_time_record))


    # obs = env.reset()
# for i in range(1000):
#     action, _states = model.predict(obs)
#     obs, rewards, dones, info = env.step(action)
#     env.render()