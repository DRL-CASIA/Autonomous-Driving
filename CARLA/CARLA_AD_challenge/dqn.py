import math
import torch
import torch.nn as nn
import numpy as np
from collections import namedtuple
from torch.utils.data.sampler import BatchSampler, SubsetRandomSampler
from tensorboardX import SummaryWriter

seed = 1
torch.manual_seed(seed)

Transition = namedtuple('Transition', ['state', 'action', 'reward', 'next_state'])

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

# networks deminsion needs to be fixed
class Net(nn.Module):
    """docstring for Net"""
    def __init__(self,input_shape,n_actions):
        super(Net, self).__init__()

        self.conv = nn.Sequential(
            nn.Conv2d(input_shape[0],16,kernel_size=4,stride=3),
            nn.ReLU(),
            nn.Conv2d(16,16,kernel_size=4,stride=2),
            nn.ReLU(),
            nn.Conv2d(16,16,kernel_size=3,stride=2),
            nn.ReLU()
        )

        self.conv_out_size = self._get_conv_out(input_shape)

        self.fc = nn.Sequential(
            nn.Linear(self.conv_out_size, 512),
            nn.ReLU(),
            nn.Linear(512, n_actions)
        )

    def _get_conv_out(self, shape):
        o = self.conv(torch.zeros(1,*shape))
        # flatten size
        return int(np.prod(o.size())) + 3

    def forward(self, image,speedx,speedy,steer):
        # conv_out = self.conv(x_tensor).view(self.conv_out_size, -1)
        image = image.to(device)
        conv_out = self.conv(image).view(image.size()[0], -1)
        speedx = speedx.to(device)
        speedy = speedy.to(device)
        steer = steer.to(device)

        input_tensor = torch.cat((conv_out,speedx,speedy,steer),1).to(device)
        return self.fc(input_tensor)



class DQNAlgorithm(object):


    capacity = 800
    learning_rate = 1e-3
    memory_counter = 0
    batch_size = 400
    gamma = 0.995
    update_count = 0
    episilo = 0.9
    dqn_epoch = 10

    episode = 0

    def __init__(self, image_shape, action_shape):

        self.image_shape = image_shape
        self.action_shape = action_shape
        
        self.eval_net = Net(self.image_shape, self.action_shape).to(device)
        self.target_net = Net(self.image_shape, self.action_shape).to(device)
        self.learn_step_counter = 0
        self.memory_counter = 0
        self.memory = [None]*self.capacity
        self.loss_func = nn.MSELoss()
        self.writer = SummaryWriter('./DQN/logs/100eps')
        self.path = '/home/guoyoutian/scenario_runner-0.9.5/DQN/'
        self.total_reward = 0.0
        self.episode_reward = 0.0
        self.episode_index = 0

    def select_action(self, input_image,speedx,speedy,steer):
        if np.random.randn() <= self.episilo:# greedy policy
            action_value = self.eval_net.forward(input_image,speedx,speedy,steer)
            action_value = action_value.to("cpu")
            action = torch.max(action_value, 1)[1].data.numpy()
            action = action[0]
        else: # random policy
            action = np.random.randint(0, self.action_shape)
        return action


    def store_transition(self,transition):
        index = self.memory_counter % self.capacity
        self.memory[index] = transition
        self.memory_counter += 1
        self.total_reward += transition.reward
    
    def action_reward(self,reward):
        self.reward = reward

    def change_rate(self,episode_index):
        self.episode = episode_index
        epsilo_start = 0.85
        epsilo_final = 0.95
        epsilo_decay = 100

        self.episilo = epsilo_start + (epsilo_final - epsilo_start) * math.exp(-1. * episode_index / epsilo_decay)


    def update(self):
        # 每个episode结束，清零total_reward
        print('episode_total_reward:',self.total_reward)
        if self.total_reward > 1200:
            self.learning_rate = 1e-4

        self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr= self.learning_rate)

        self.episode_reward += self.total_reward
        self.total_reward = 0.0
        self.episode_index += 1
        if self.episode_index % 10 == 0:
            mean_reward_10 = self.episode_reward /10
            index = self.episode_index / 10
            self.writer.add_scalar('mean_reward_10', mean_reward_10, index)
            self.episode_reward = 0

        print('episode:',self.episode)
        if self.memory_counter > self.capacity:
            with torch.no_grad():
            
                state_image = torch.tensor([t.state['image'] for t in self.memory]).float().to(device)
                state_speedx = torch.tensor([t.state['speedx'] for t in self.memory]).float().to(device)
                state_speedx = state_speedx.unsqueeze(1)
                state_speedy = torch.tensor([t.state['speedy'] for t in self.memory]).float().to(device)
                state_speedy = state_speedy.unsqueeze(1)
                state_steer = torch.tensor([t.state['steer'] for t in self.memory]).float().to(device)
                state_steer = state_steer.unsqueeze(1)

                action = torch.LongTensor([t.action for t in self.memory]).view(-1,1).long().to(device)
                reward = torch.tensor([t.reward for t in self.memory]).float().to(device)

                next_state_image = torch.tensor([t.next_state['image'] for t in self.memory]).float().to(device)
                next_state_speedx = torch.tensor([t.next_state['speedx'] for t in self.memory]).float().to(device)
                next_state_speedx = next_state_speedx.unsqueeze(1)
                next_state_speedy = torch.tensor([t.next_state['speedy'] for t in self.memory]).float().to(device)
                next_state_speedy = next_state_speedy.unsqueeze(1)
                next_state_steer = torch.tensor([t.next_state['steer'] for t in self.memory]).float().to(device)
                next_state_steer = next_state_steer.unsqueeze(1)

            
                reward = (reward - reward.mean()) / (reward.std() + 1e-7)

                target_v = reward + self.gamma * self.target_net(next_state_image,next_state_speedx,next_state_speedy,next_state_steer).max(1)[0]

            #Update...
            for _ in range(self.dqn_epoch): # iteration ppo_epoch 
                for index in BatchSampler(SubsetRandomSampler(range(len(self.memory))), batch_size=self.batch_size, drop_last=False):
                    # v = (self.eval_net(state_image,state_speedx,state_speedy,state_steer).gather(1, action))[index]
                    loss = self.loss_func(target_v[index].unsqueeze(1), (self.eval_net(state_image,state_speedx,state_speedy,state_steer).gather(1, action))[index])

                    self.optimizer.zero_grad()
                    loss.backward()
                    self.optimizer.step()
                    self.writer.add_scalar('loss/value_loss', loss, self.update_count)
                    self.update_count +=1
                    if self.update_count % 100 ==0:
                        self.target_net.load_state_dict(self.eval_net.state_dict())

            #self.memory_counter += 1
        else:
            print("Memory Buff is too less")




    def save_net(self):
        print('enter save')
        import pdb
        pdb.set_trace()
        torch.save(self.eval_net.state_dict(), self.path +'dqn.pth')

    def load_net(self):
        import pdb
        pdb.set_trace()
        self.eval_net.load_state_dict(torch.load(self.path +'dqn.pth'))
        self.target_net.load_state_dict(torch.load(self.path +'dqn.pth'))