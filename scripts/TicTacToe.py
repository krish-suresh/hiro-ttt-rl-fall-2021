#!/usr/bin/env python
from math import pi
from tf.transformations import quaternion_from_euler
from hiro_core.XamyabRobot import XamyabRobot, rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion


# roslaunch ur_gazebo xamyab.launch
# roslaunch xamyab_moveit_config xamyab_moveit_planning_execution.launch

class TicTacToe:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=True)
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, pi / 2))
        pose_goal = Pose(position=Point(*[0.6256, -0.50, 0.2]), orientation=self.default_gripper_quaternion)
        self.robot.right_manipulator.set_pose_goal(pose_goal)
    def play(self):

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