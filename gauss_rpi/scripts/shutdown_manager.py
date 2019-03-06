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
import threading 

from gauss_rpi.rpi_ros_utils import send_shutdown_command 

from gauss_msgs.srv import SetInt


class ShutdownManager: 
    
    def callback_shutdown_rpi(self, req):
        if req.value==1:  
            send_shutdown_command_thread = threading.Timer(1.0,send_shutdown_command)
            send_shutdown_command_thread.start()
            return {'status': 200, 'message': 'Robot is shutting down'}
        return {'status': 400, 'message': 'Robot is not shutting down : try request value :1 to shutdown rpi'}

    def __init__(self):
        self.shutdown_rpi_sever=rospy.Service('/gauss/rpi/shutdown_rpi', SetInt, self.callback_shutdown_rpi) 
        rospy.loginfo("Shutdown Manager OK")


