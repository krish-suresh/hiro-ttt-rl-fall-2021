#!/usr/bin/env python
from robot import Robot
from hiro_core.XamyabRobot import XamyabRobot, rospy


class TicTacToe:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=True)
    def play():
        # Draw Board
        # Randomize Starting player
        # loop while game not done:
        #     game.state = robot.playMove(model(game.state), isX: True)     
        #     game.state = robot.playMove(userMove, isX: False)
        # output win/lose/tie
        pass
    def drawBoard(self):
        pass
    def playMove(self, pos, isX):
        pass

if __name__ == "__main__":
    tictactoe = TicTacToe()
    tictactoe.play()