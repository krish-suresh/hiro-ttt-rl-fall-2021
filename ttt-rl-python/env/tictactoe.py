import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
import tensorflow as tf
class TicTacToe(gym.Env):
    metadata = {"render.modes": ["human", "rgb_array"], "video.frames_per_second": 50}
    # Board Indices
    # 0 1 2
    # 3 4 5
    # 6 7 8
    # 
    # below are the values that represent our values for the matrix
    # 0 = empty
    # 1 = O
    # 2 = X
    # Possible Game
    # 1 | 2 | 0
    # ----------
    # 0 | 1 | 2
    # ----------
    # 2 | 1 | 1
    # O wins

    
    def __init__(self):
        super(TicTacToe, self).__init__()
        self.action_space = spaces.Discrete(9)
        self.observation_shape = (3, 3)
        self.observation_space = spaces.Box(low = np.zeros(self.observation_shape), 
                                            high = np.ones(self.observation_shape)*2,
                                            dtype = np.int8)
        self.state = None
    def step(self, action, player):
        if self.is_valid_move(action):
            self.apply_move(action, player)
            if self.is_won():
                reward = 1
                done = True
            elif self.is_lost():
                reward = -1
                done = True
            elif self.is_done():
                reward = 0.5
                done = True
            else:
                reward = 0
                done = False
        else:
            done = True
            reward = -1
        return self.state, reward, done
    def invert(self):
        inverted = tf.Variable(tf.reshape(self.state, [1,9]))
        for i in range(0, 9):
            if inverted[0, i] == 1:
                inverted[0, i].assign(2)
            elif inverted[0, i] == 2:
                inverted[0, i].assign(1)
        inverted = tf.reshape(inverted, [3, 3])
        return inverted
    def reset(self):
        self.state = tf.Variable(tf.zeros([3, 3], tf.int8))
        
        return self.state
    def render(self, mode='human'):
        ...
    def close (self):
        ...
    def is_valid_move(self, action):
        return tf.reshape(self.state, [1, 9])[0, action] == 0
    def is_done(self):
        blah = tf.reshape(self.state, [1,9]) #this was ayush's coding decision to name it blah, raiyan advocated for temp
        return (blah[0,0] != 0 and blah[0,1] != 0 and blah[0,2] != 0 and blah[0,3] != 0 and blah[0,4] != 0 and blah[0,5] != 0 and blah[0,6] != 0 and blah[0,7] != 0 and blah[0,8] != 0)
    def apply_move(self, action, player):
        self.state = tf.Variable(tf.reshape(self.state,[1,9]))
        self.state = self.state[0,action].assign(player)
        self.state = tf.reshape(self.state, [3,3])
        return self.state
    def is_won(self):
        self.state = tf.Variable(tf.reshape(self.state,[1,9]))
        if (self.state[0, 0] == self.state[0, 1] == self.state[0, 2] == 1):
            return True
        if (self.state[0, 3] == self.state[0, 4] == self.state[0, 5] == 1):
            return True
        if (self.state[0, 6] == self.state[0, 7] == self.state[0, 8] == 1):
            return True
        if (self.state[0, 0] == self.state[0, 3] == self.state[0, 6] == 1):
            return True
        if (self.state[0, 1] == self.state[0, 4] == self.state[0, 7] == 1):
            return True
        if (self.state[0, 2] == self.state[0, 5] == self.state[0, 8] == 1):
            return True
        if (self.state[0, 0] == self.state[0, 4] == self.state[0, 8] == 1):
            return True
        if (self.state[0, 2] == self.state[0, 4] == self.state[0, 6] == 1):
            return True   
        else:     
            return False
    def is_lost(self):
        self.state = tf.Variable(tf.reshape(self.state,[1,9]))
        if (self.state[0, 0] == self.state[0, 1] == self.state[0, 2] == 2):
            return True
        if (self.state[0, 3] == self.state[0, 4] == self.state[0, 5] == 2):
            return True
        if (self.state[0, 6] == self.state[0, 7] == self.state[0, 8] == 2):
            return True
        if (self.state[0, 0] == self.state[0, 3] == self.state[0, 6] == 2):
            return True
        if (self.state[0, 1] == self.state[0, 4] == self.state[0, 7] == 2):
            return True
        if (self.state[0, 2] == self.state[0, 5] == self.state[0, 8] == 2):
            return True
        if (self.state[0, 0] == self.state[0, 4] == self.state[0, 8] == 2):
            return True
        if (self.state[0, 2] == self.state[0, 4] == self.state[0, 6] == 2):
            return True   
        else:     
            return False