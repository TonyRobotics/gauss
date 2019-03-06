#!/usr/bin/env python

# Copyright (C) 2017 Niryo
# All rights reserved.
#
# This program is free software: you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation, either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <http://www.gnu.org/licenses/>.

import rospy
from position_manager import PositionManager
from trajectory_manager import TrajectoryManager
from gauss_robot_state_publisher import GaussRobotStatePublisher
from robot_commander import RobotCommander 

from gauss_commander.gauss_ros_logger import RosLogger

class GaussCommanderNode(): 

    def __init__(self): 
        #init RosLogger
        self.debug_mode = rospy.get_param("~debug_mode")
        self.gauss_ros_logger = RosLogger(debug_mode = self.debug_mode)
        
        # Publish robot state (position, orientation, tool)
        self.gauss_robot_state_publisher = GaussRobotStatePublisher()

        self.simulator_mode = rospy.get_param("~simulator_mode")

        # Position Manager  
        positions_dir = rospy.get_param("~positions_dir")
        self.pos_manager = PositionManager(positions_dir, self.gauss_ros_logger)
        #trajectory_manager 
        trajectories_dir = rospy.get_param("~trajectories_dir")
        self.traj_manager = TrajectoryManager(trajectories_dir, self.gauss_ros_logger)
        # robot commander 
        self.robot_commander = RobotCommander(self.simulator_mode, self.pos_manager, self.traj_manager, self.gauss_ros_logger)
        self.robot_commander.start()

        self.gauss_ros_logger.publish_log_status("INFO", "GaussCommanderNode start over")

if __name__ == '__main__':
    rospy.init_node('gauss_commander')
    gauss_commander_node = GaussCommanderNode()
    rospy.spin()

