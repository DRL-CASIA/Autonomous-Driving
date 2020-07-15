import random
import numpy as np
from collections import deque

class ReplayMemory(object):
    """docstring for ReplayBuffer"""
    def __init__(self, size, random_seed=123):
        self.size = size
        self.count = 0
        self.buffer = deque()
        # random.seed(random_seed)

    def add(self, s, a, r, s2, t):
        experience = (s,a,r,s2,t)
        if self.count < self.size:
            self.buffer.append(experience)
            self.count += 1
        else:
            self.buffer.popleft()
            self.buffer.append(experience)

    def size(self):
        return self.count

    def get_minibatch(self, batch_sz):
        batch = []
        if self.count < batch_sz:
            batch = random.sample(self.buffer, self.count)
        else:
            batch = random.sample(self.buffer, batch_sz)

        s = np.array([e[0] for e in batch])
        a = np.array([e[1] for e in batch])
        r = np.array([e[2] for e in batch])
        s2 = np.array([e[3] for e in batch])
        t = np.array([e[4] for e in batch])
        return (s,a,r,s2,t)
