#! /usr/bin/env python

import roslib
# roslib.load_manifest('actionlib_test')
import rospy
import actionlib

from gauss_msgs.msg import RobotMoveAction
from gauss_msgs.msg import RobotMoveGoal
from gauss_msgs.msg import RobotMoveResult
from gauss_commander.command_type import CommandType as MoveCommandType

if __name__ == '__main__':
    rospy.init_node('robot_action_client')  

    client = actionlib.SimpleActionClient('/gauss/commander/robot_action', RobotMoveAction)
    if not client.wait_for_server(rospy.Duration(5.0)):
        exit
    goal = RobotMoveGoal()
    goal.cmd.cmd_type = MoveCommandType.JOINTS
    goal.cmd.joints = [0,0,0,0,0,0]
    client.send_goal(goal)
    client.wait_for_result(rospy.Duration.from_sec(10.0))
    result = client.get_result()