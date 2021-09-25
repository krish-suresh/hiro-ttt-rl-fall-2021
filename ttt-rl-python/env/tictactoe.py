import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np

class TicTacToe(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}
    # Board Indices
    # 0 1 2
    # 3 4 5
    # 6 7 8
    def __init__(self):
        super(TicTacToe, self).__init__()
        self.action_space = spaces.Discrete(9)
        self.observation_shape = (3, 3, 3)
        self.observation_space = spaces.Box(low = np.zeros(self.observation_shape), 
                                            high = np.ones(self.observation_shape),
                                            dtype = np.int8)
        self.state = None
    def step(self, action):
        if self.is_valid_move(action):
            self.apply_move(action)
            if self.won():
                reward = 1
                done = True
            elif self.lost():
                reward = 0
                done = True
            else:
                reward = 0
        else:
            done = True
        return observation, reward, done, info
    def reset(self):
        ...
        return observation
    def render(self, mode='human'):
        ...
    def close (self):
        ...
    def is_valid_move(action):
        return False
    def apply_move(action):
        return False
    def won(action):
        return False