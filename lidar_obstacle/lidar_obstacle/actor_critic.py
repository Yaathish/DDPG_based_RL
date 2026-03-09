import torch
import torch.nn as nn
import numpy as np

# ─────────────────────────────────────────────────────────────────────
#  Observation = 24 LiDAR bins + distance_to_goal + angle_to_goal = 26
#  Action      = [linear_vel, angular_vel]  both in [-1, 1]
# ─────────────────────────────────────────────────────────────────────
STATE_DIM  = 26
ACTION_DIM = 2
MAX_ACTION = 1.0


class Actor(nn.Module):
    def __init__(self, state_dim=STATE_DIM, action_dim=ACTION_DIM, max_action=MAX_ACTION):
        super(Actor, self).__init__()
        self.max_action = max_action
        self.net = nn.Sequential(
            nn.Linear(state_dim, 400),
            nn.ReLU(),
            nn.Linear(400, 300),
            nn.ReLU(),
            nn.Linear(300, action_dim),
            nn.Tanh()
        )

    def forward(self, state):
        return self.max_action * self.net(state)


class Critic(nn.Module):
    def __init__(self, state_dim=STATE_DIM, action_dim=ACTION_DIM):
        super(Critic, self).__init__()
        self.q1 = nn.Sequential(
            nn.Linear(state_dim + action_dim, 400), nn.ReLU(),
            nn.Linear(400, 300),                    nn.ReLU(),
            nn.Linear(300, 1)
        )
        self.q2 = nn.Sequential(
            nn.Linear(state_dim + action_dim, 400), nn.ReLU(),
            nn.Linear(400, 300),                    nn.ReLU(),
            nn.Linear(300, 1)
        )

    def forward(self, state, action):
        sa = torch.cat([state, action], dim=1)
        return self.q1(sa), self.q2(sa)

    def Q1(self, state, action):
        sa = torch.cat([state, action], dim=1)
        return self.q1(sa)


class ReplayBuffer:
    def __init__(self, state_dim=STATE_DIM, action_dim=ACTION_DIM, max_size=300_000):
        self.max_size = max_size
        self.ptr      = 0
        self.size     = 0
        self.state      = np.zeros((max_size, state_dim),  dtype=np.float32)
        self.action     = np.zeros((max_size, action_dim), dtype=np.float32)
        self.next_state = np.zeros((max_size, state_dim),  dtype=np.float32)
        self.reward     = np.zeros((max_size, 1),          dtype=np.float32)
        self.done       = np.zeros((max_size, 1),          dtype=np.float32)

    def add(self, state, action, next_state, reward, done):
        self.state[self.ptr]      = state
        self.action[self.ptr]     = action
        self.next_state[self.ptr] = next_state
        self.reward[self.ptr]     = reward
        self.done[self.ptr]       = done
        self.ptr  = (self.ptr + 1) % self.max_size
        self.size = min(self.size + 1, self.max_size)

    def sample(self, batch_size=256):
        idx    = np.random.randint(0, self.size, size=batch_size)
        device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
        return (
            torch.FloatTensor(self.state[idx]).to(device),
            torch.FloatTensor(self.action[idx]).to(device),
            torch.FloatTensor(self.next_state[idx]).to(device),
            torch.FloatTensor(self.reward[idx]).to(device),
            torch.FloatTensor(self.done[idx]).to(device),
        )

    def __len__(self):
        return self.size