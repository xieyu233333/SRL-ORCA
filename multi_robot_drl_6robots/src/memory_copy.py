import numpy as np
import random
from datetime import datetime
import torch

class ReplayBuffer:
    def __init__(self, capacity):
        self.capacity = capacity
        self.buffer = []
        self.position = 0
    
    def push(self, state, action, reward, next_state, done):
        if len(self.buffer) < self.capacity:
            self.buffer.append(None)
        self.buffer[self.position] = (state, action, reward, next_state, done)
        self.position = (self.position + 1) % self.capacity
    
    def sample(self, batch_size):
        batch = random.sample(self.buffer, batch_size)
        state, action, reward, next_state, done = map(np.stack, zip(*batch))
        return state, action, reward, next_state, done
    
    def __len__(self):
        return len(self.buffer)

class ReplayBuffer2(object):
    def __init__(self, state_dim, action_dim, size):
        self.max_size = size
        self.count = 0
        self.size = 0
        self.s = np.zeros((self.max_size, state_dim))
        self.a = np.zeros((self.max_size, action_dim))
        self.r = np.zeros((self.max_size))
        self.s_ = np.zeros((self.max_size, state_dim))
        self.dw = np.zeros((self.max_size))

    def push(self, s, a, r, s_, dw):
        self.s[self.count] = s
        self.a[self.count] = a
        self.r[self.count] = r
        self.s_[self.count] = s_
        self.dw[self.count] = dw
        self.count = (self.count + 1) % self.max_size  # When the 'count' reaches max_size, it will be reset to 0.
        self.size = min(self.size + 1, self.max_size)  # Record the number of  transitions

    def sample(self, batch_size):
        if batch_size > self.size:
            batch_size = self.size
        index = np.random.choice(self.size, size=batch_size)  # Randomly sampling
        # print(index)
        batch_s = torch.tensor(self.s[index], dtype=torch.float)
        batch_a = torch.tensor(self.a[index], dtype=torch.float)
        batch_r = torch.tensor(self.r[index], dtype=torch.float)
        batch_s_ = torch.tensor(self.s_[index], dtype=torch.float)
        batch_dw = torch.tensor(self.dw[index], dtype=torch.float)

        return batch_s, batch_a, batch_r, batch_s_, batch_dw

class ReplayBuffer10:
    def __init__(self, capacity):
        self.capacity = capacity # capacity of the buffer
        self.data = []
        self.index = 0 # index of the next cell to be filled
    def push(self, s, a, r, s_, d):
        if len(self.data) < self.capacity:
            self.data.append(None)
        self.data[self.index] = (s, a, r, s_, d)
        self.index = (self.index + 1) % self.capacity
    def sample(self, batch_size):
        batch = random.sample(self.data, batch_size)
        return list(map(np.array, list(zip(*batch))))
    def __len__(self):
        return len(self.data)

state_dimension = 400
action_dimension = 2
cap = 50000
buf = ReplayBuffer(cap)
# buf = ReplayBuffer2(state_dimension, action_dimension,cap)
# buf = ReplayBuffer10(cap)

for i in range(50000):
    state = np.random.rand(state_dimension)
    next_state = np.random.rand(state_dimension)
    action = np.random.rand(2)
    reward = np.random.rand(1)
    done = np.round(np.random.rand(1))

    # start_t = datetime.now()
    buf.push(state, action, reward, next_state, done)
    # print((datetime.now() - start_t).microseconds)

for i in range(10):
    start_t = datetime.now()
    batch_s, batch_a, batch_r, batch_s_, batch_dw = buf.sample(1000)
    batch_s = torch.tensor(batch_s)
    batch_s_ = torch.tensor(batch_s_)
    batch_a = torch.tensor(batch_a)
    batch_r = torch.tensor(batch_r)
    batch_dw = torch.tensor(batch_dw)
    print('  ', type(batch_s))
    print((datetime.now() - start_t).microseconds)