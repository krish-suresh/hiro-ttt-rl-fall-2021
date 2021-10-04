#!/usr/bin/env python
from math import pi
from tf.transformations import quaternion_from_euler
from hiro_core.XamyabRobot import XamyabRobot, rospy
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
import random

# roslaunch ur_gazebo xamyab.launch
# roslaunch xamyab_moveit_config xamyab_moveit_planning_execution.launch

class TicTacToe:
    def __init__(self):
        self.robot = XamyabRobot(visualize_trajectory=True)
        self.default_gripper_quaternion = Quaternion(*quaternion_from_euler(pi, 0, 0))
        self.board_pos = Pose(position=Point(*[0.6256, 0, 0.4]), orientation=self.default_gripper_quaternion)
        self.board_size = 0.15
        self.shape_size = self.board_size/4
        self.robot.right_manipulator.home()
        self.robot.right_gripper.close()
        self.move_to(self.board_pos)
    def reset(self):
        self.robot.left_gripper.open()
        rospy.loginfo("Going home")
        self.robot.left_manipulator.home()

    def set_up_environment(self, object_name, transform_point):
        object_pose = PoseStamped()
        object_pose.header.frame_id = self.robot.left_manipulator.get_planning_frame()
        rospy.loginfo("Object {} position {}.".format(object_name, transform_point))
        object_pose.pose = Pose(position=Point(*transform_point))
        self.robot.left_gripper.enable_fingers_collisions(object_name, True)
    def move_to_point(self, point_goal):
        self.move_to(Pose(position=point_goal, orientation=self.default_gripper_quaternion))
    def move_to(self, pose_goal):
        pose_goal.orientation = self.default_gripper_quaternion
        self.robot.right_manipulator.set_pose_goal(pose_goal)
        self.robot.right_manipulator.go(wait=True)
        self.robot.right_manipulator.stop()
        self.robot.right_manipulator.clear_pose_targets()
    def follow_path(self, waypoints):
        (plan, fraction) = self.robot.right_manipulator.compute_cartesian_path(
                                   waypoints,   # waypoints to follow
                                   0.01,        # eef_step
                                   0.0)         # jump_threshold
        self.robot.right_manipulator.execute(plan, wait=True)
    def test(self):
        self.drawBoard()
        self.drawMove(0, True)
        self.drawMove(1, False)
        self.drawMove(2, True)    
    def play(self):
        # Draw Board
        self.drawBoard()
        # Randomize Starting player
        isRobotStart = bool(random.getrandbits(1))
        # loop while game not done:
        #     game.state = robot.playMove(model(game.state), isX: True)     
        #     game.state = robot.playMove(userMove, isX: False)
        # output win/lose/tie

        # done = False
        # while not done:
        #     pass
    def drawBoard(self):
        rospy.loginfo("Drawing Board")
        boardLines = self.getBoardLines()
        for line in boardLines:
            self.drawLine(line)

    def drawMove(self, pos, isX):
        drawPos = self.getDrawPosFromBoardPos(pos)
        if isX:
            self.drawX(drawPos)
        else:
            self.drawO(drawPos)
        pass
    def drawLine(self, line):
        startPose = line[0]
        endPose = line[1]
        # move to startPose with elevated z
        startPose.z += 0.01
        self.move_to_point(startPose)
        # move to startPose
        startPose.z -= 0.01
        self.move_to_point(startPose)
        # path line to endPose
        self.move_to_point(endPose)
        # move to endPose with elevated z
        endPose.z += 0.01
        self.move_to_point(endPose)
    def getBoardLines(self):
        left = []
        left.append(Point(self.board_pos.position.x+self.board_size/2, self.board_pos.position.y+self.board_size/6, self.board_pos.position.z))
        left.append(Point(self.board_pos.position.x-self.board_size/2, self.board_pos.position.y+self.board_size/6, self.board_pos.position.z))
        right = []
        right.append(Point(self.board_pos.position.x+self.board_size/2, self.board_pos.position.y-self.board_size/6, self.board_pos.position.z))
        right.append(Point(self.board_pos.position.x-self.board_size/2, self.board_pos.position.y-self.board_size/6, self.board_pos.position.z))
        top = []
        top.append(Point(self.board_pos.position.x+self.board_size/6, self.board_pos.position.y+self.board_size/2, self.board_pos.position.z))
        top.append(Point(self.board_pos.position.x+self.board_size/6, self.board_pos.position.y-self.board_size/2, self.board_pos.position.z))
        bottom = []
        bottom.append(Point(self.board_pos.position.x-self.board_size/6, self.board_pos.position.y+self.board_size/2, self.board_pos.position.z))
        bottom.append(Point(self.board_pos.position.x-self.board_size/6, self.board_pos.position.y-self.board_size/2, self.board_pos.position.z))
        return [left, right, top, bottom]
    def getDrawPosFromBoardPos(self, boardPos):
        pose = Point()
        pose.z = self.board_pos.position.z
        if boardPos in [0, 1, 2]:
            pose.y = self.board_pos.position.y+self.board_size/3
        elif boardPos in [3, 4, 5]:
            pose.y = self.board_pos.position.y
        elif boardPos in [6, 7, 8]:
            pose.y = self.board_pos.position.y-self.board_size/3
        if boardPos in [0, 3, 6]:
            pose.x = self.board_pos.position.x-self.board_size/3
        elif boardPos in [1, 4, 7]:
            pose.x = self.board_pos.position.x
        elif boardPos in [2, 5, 8]:
            pose.x = self.board_pos.position.x+self.board_size/3
        return pose
    def drawO(self, centerPose):
        # draw a circle of size self.shape_size
        waypoints = []
        # TODO calculate circle waypoints
        self.follow_path(waypoints)
        pass
    def drawX(self, centerPose):
        # get X lines from centerPose and boardSize
        centerOffset = self.shape_size
        line0 = []
        line0[0] = Point(centerPose.x-centerOffset, centerPose.y+centerOffset, self.board_pos.position.z)
        line0[1] = Point(centerPose.x+centerOffset, centerPose.y-centerOffset, self.board_pos.position.z)
        line1 = []
        line1[0] = Point(centerPose.x+centerOffset, centerPose.y+centerOffset, self.board_pos.position.z)
        line1[1] = Point(centerPose.x-centerOffset, centerPose.y-centerOffset, self.board_pos.position.z)
        x_lines = [line0, line1]
        for line in x_lines:
            self.drawLine(line)
        pass
if __name__ == "__main__":
    tictactoe = TicTacToe()
    tictactoe.test()