import math
import gym
from gym import spaces, logger
from gym.utils import seeding
import numpy as np
import tensorflow as tf

from tictactoe import TicTacToe
test = TicTacToe()
test.reset()
print(test.step(0, 2))
print(test.step(4, 1))
print(test.step(7, 2))
print(test.step(2, 1))
print(test.step(6, 2))
print(test.step(8, 1))
print(test.step(3, 2))
print(test.invert())