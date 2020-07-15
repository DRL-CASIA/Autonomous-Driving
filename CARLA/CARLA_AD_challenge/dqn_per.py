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


class NaivePrioritizedBuffer(object):
    def __init__(self, capacity, prob_alpha=0.6):
        self.prob_alpha = prob_alpha
        self.capacity   = capacity
        self.buffer     = []
        self.pos        = 0
        self.priorities = np.zeros((capacity,), dtype=np.float32)
    
    def push(self, transition):
        # state      = np.expand_dims(state, 0)
        # next_state = np.expand_dims(next_state, 0)
        
        max_prio = self.priorities.max() if self.buffer else 1.0
        
        if len(self.buffer) < self.capacity:
            self.buffer.append(transition)
        else:
            self.buffer[self.pos] = (transition)
        
        self.priorities[self.pos] = max_prio
        self.pos = (self.pos + 1) % self.capacity
    
    def sample(self, batch_size, beta=0.4):
        if len(self.buffer) == self.capacity:
            prios = self.priorities
        else:
            prios = self.priorities[:self.pos]
        
        probs  = prios ** self.prob_alpha
        probs /= probs.sum()
        
        indices = np.random.choice(len(self.buffer), batch_size, p=probs)
        
        total    = len(self.buffer)
        weights  = (total * probs[indices]) ** (-beta)
        weights /= weights.max()
        weights  = np.array(weights, dtype=np.float32)
        
        return indices, weights
    
    def update_priorities(self, batch_indices, batch_priorities):

        for idx, prio in zip(batch_indices, batch_priorities):
            self.priorities[idx] = prio

    def __len__(self):
        return len(self.buffer)

# networks deminsion needs to be fixed
class Net(nn.Module):
    """docstring for Net"""
    def __init__(self,input_shape,n_actions):
        super(Net, self).__init__()

        self.conv = nn.Sequential(
            nn.Conv2d(input_shape[0],16,kernel_size=4,stride=3),
            nn.ReLU(),
            nn.Conv2d(16,16,kernel_size=3,stride=2),
            nn.ReLU(),
            nn.Conv2d(16,16,kernel_size=3,stride=2),
            nn.ReLU()
        )

        self.conv_out_size = self._get_conv_out(input_shape)

        self.fc = nn.Sequential(
            nn.Linear(self.conv_out_size, 1024),
            nn.LeakyReLU(),
            nn.Linear(1024, n_actions)
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



class DQN_PERAlgorithm(object):


    capacity = 800
    learning_rate = 1e-3
    memory_counter = 0
    batch_size = 300
    gamma = 0.995
    update_count = 0
    episilo = 0.9
    beta = 0.4
    dqn_epoch = 10

    episode = 0

    finish_count =0 

    def __init__(self, image_shape, action_shape):

        self.image_shape = image_shape
        self.action_shape = action_shape
        
        self.eval_net = Net(self.image_shape, self.action_shape).to(device)
        self.target_net = Net(self.image_shape, self.action_shape).to(device)
        self.learn_step_counter = 0
        self.memory_counter = 0
        # self.memory = [None]*self.capacity
        self.memory = NaivePrioritizedBuffer(self.capacity)

        self.loss_func = nn.MSELoss(reduce = False)
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
        self.memory.push(transition)
        self.memory_counter += 1
        self.total_reward += transition.reward

        beta_start = 0.4
        beta_frames = 100000
        self.beta = min(1.0, beta_start + self.memory_counter * (1.0 - beta_start) / beta_frames)
    
    def action_reward(self,reward):
        self.reward = reward

    def change_rate(self,episode_index):
        self.episode = episode_index
        epsilo_start = 0.9
        epsilo_final = 0.95
        epsilo_decay = 100

        self.episilo = epsilo_start + (epsilo_final - epsilo_start) * math.exp(-1. * episode_index / epsilo_decay)


    def update(self):
        # 每个episode结束，清零total_reward
        print('episode_total_reward:',self.total_reward)
        if self.total_reward > 1200:
            self.learning_rate = 1e-5
            self.save_net()

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
            
            state_image = torch.tensor([t.state['image'] for t in self.memory.buffer]).float().to(device)
            state_speedx = torch.tensor([t.state['speedx'] for t in self.memory.buffer]).float().to(device)
            state_speedx = state_speedx.unsqueeze(1)
            state_speedy = torch.tensor([t.state['speedy'] for t in self.memory.buffer]).float().to(device)
            state_speedy = state_speedy.unsqueeze(1)
            state_steer = torch.tensor([t.state['steer'] for t in self.memory.buffer]).float().to(device)
            state_steer = state_steer.unsqueeze(1)

            action = torch.LongTensor([t.action for t in self.memory.buffer]).view(-1,1).long().to(device)
            reward = torch.tensor([t.reward for t in self.memory.buffer]).float().to(device)

            next_state_image = torch.tensor([t.next_state['image'] for t in self.memory.buffer]).float().to(device)
            next_state_speedx = torch.tensor([t.next_state['speedx'] for t in self.memory.buffer]).float().to(device)
            next_state_speedx = next_state_speedx.unsqueeze(1)
            next_state_speedy = torch.tensor([t.next_state['speedy'] for t in self.memory.buffer]).float().to(device)
            next_state_speedy = next_state_speedy.unsqueeze(1)
            next_state_steer = torch.tensor([t.next_state['steer'] for t in self.memory.buffer]).float().to(device)
            next_state_steer = next_state_steer.unsqueeze(1)

        
            reward = (reward - reward.mean()) / (reward.std() + 1e-7)
            with torch.no_grad():
                target_v = reward + self.gamma * self.target_net(next_state_image,next_state_speedx,next_state_speedy,next_state_steer).max(1)[0]

            #Update...
            for _ in range(self.dqn_epoch): # iteration ppo_epoch 
                
                indices, weights = self.memory.sample(self.batch_size,self.beta)

                weights = torch.tensor(weights).to(device)

                loss = (target_v[indices].unsqueeze(1).squeeze(1) - (self.eval_net(state_image,state_speedx,state_speedy,state_steer).gather(1, action))[indices].squeeze(1)).pow(2)* weights
                # loss = self.loss_func(target_v[indices].unsqueeze(1), (self.eval_net(state_image,state_speedx,state_speedy,state_steer).gather(1, action))[indices])   
                prios = loss + 1e-5
                loss  = loss.mean()
       
                self.optimizer = torch.optim.Adam(self.eval_net.parameters(), lr= self.learning_rate)
                self.optimizer.zero_grad()
                loss.backward()
                self.memory.update_priorities(indices, prios.data.cpu().numpy())
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