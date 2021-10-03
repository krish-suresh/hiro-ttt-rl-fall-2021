#!/usr/bin/env python
from robot import Robot
from hiro_core.XamyabRobot import XamyabRobot, rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

class TicTacToe:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=True)
    def play(self):
        pose_goal = Pose()
        pose_goal.orientation.w = 1.0
        pose_goal.position.x = 0.4
        pose_goal.position.y = 0.1
        pose_goal.position.z = 0.4
        self.robot.right_manipulator.set_pose_goal()
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
    input("Move Robot")
    tictactoe.play()