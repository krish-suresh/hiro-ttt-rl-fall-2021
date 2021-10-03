import math

import numpy as np
import tensorflow as tf
class TicTacToe():
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

    tf.random.set_seed(42)

    def __init__(self):
        super(TicTacToe, self).__init__()
        self.state = None
    def step(self, action, player):
        if self.is_valid_move(action):
            self.apply_move(action, player)
            if self.is_won():
                reward = 15
                print(tf.reshape(self.state, [3,3]))
                done = True
            elif self.is_lost():
                reward = -5
                print(tf.reshape(self.state, [3,3]))
                done = True
            elif self.is_done():
                reward = 20
                print("Tied")
                done = True
            else:
                reward = 1
                done = False
        else:
            print("invalid")
            done = True
            reward = -1
        # print(tf.reshape(self.state, [3,3]))
        return self.state, reward, done
    def validMoves(self):
        state = tf.Variable(self.state)
        ret = tf.Variable(tf.zeros([1, 9], tf.float32))
        for i in range(0, 9):
            if state[0,i] != 0:
                ret[0, i].assign(-1000)
        return ret
    def invert(self):
        inverted = tf.Variable(tf.reshape(self.state, [1,9]))
        for i in range(0, 9):
            if inverted[0, i] == 1:
                inverted[0, i].assign(2)
            elif inverted[0, i] == 2:
                inverted[0, i].assign(1)
        self.state = tf.reshape(inverted, [1, 9])
    def reset(self):
        self.state = tf.Variable(tf.zeros([1, 9], tf.int8))
        
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
        checkVal = 1
        if (self.state[0, 0] == checkVal and  self.state[0, 1] == checkVal and self.state[0, 2] == checkVal):
            return True
        if (self.state[0, 3] == checkVal and self.state[0, 4] == checkVal and self.state[0, 5] == checkVal):
            return True
        if (self.state[0, 6] == checkVal and self.state[0, 7] == checkVal and self.state[0, 8] == checkVal):
            return True
        if (self.state[0, 0] == checkVal and self.state[0, 3] == checkVal and self.state[0, 6] == checkVal):
            return True
        if (self.state[0, 1] == checkVal and self.state[0, 4] == checkVal and self.state[0, 7] == checkVal):
            return True
        if (self.state[0, 2] == checkVal and self.state[0, 5] == checkVal and self.state[0, 8] == checkVal):
            return True
        if (self.state[0, 0] == checkVal and self.state[0, 4] == checkVal and self.state[0, 8] == checkVal):
            return True
        if (self.state[0, 2] == checkVal and self.state[0, 4] == checkVal and self.state[0, 6] == checkVal):
            return True   
        else:     
            return False
    def is_lost(self):
        self.state = tf.Variable(tf.reshape(self.state,[1,9]))
        checkVal = -1
        if (self.state[0, 0] == checkVal and  self.state[0, 1] == checkVal and self.state[0, 2] == checkVal):
            return True
        if (self.state[0, 3] == checkVal and self.state[0, 4] == checkVal and self.state[0, 5] == checkVal):
            return True
        if (self.state[0, 6] == checkVal and self.state[0, 7] == checkVal and self.state[0, 8] == checkVal):
            return True
        if (self.state[0, 0] == checkVal and self.state[0, 3] == checkVal and self.state[0, 6] == checkVal):
            return True
        if (self.state[0, 1] == checkVal and self.state[0, 4] == checkVal and self.state[0, 7] == checkVal):
            return True
        if (self.state[0, 2] == checkVal and self.state[0, 5] == checkVal and self.state[0, 8] == checkVal):
            return True
        if (self.state[0, 0] == checkVal and self.state[0, 4] == checkVal and self.state[0, 8] == checkVal):
            return True
        if (self.state[0, 2] == checkVal and self.state[0, 4] == checkVal and self.state[0, 6] == checkVal):
            return True   
        else:     
            return False