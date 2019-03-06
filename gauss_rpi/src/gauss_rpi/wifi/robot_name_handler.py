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

import os

#
# Feature to set a name for the robot (different than "Gauss")
# - Will appear on Wi-Fi and ROS connection (todo)
#


# default value, will be replaced with value in setup launch file
FILENAME = '~/gauss_saved_values/robot_name.txt' 

def get_filename_robot_name():
    return FILENAME

def set_filename_robot_name(filename):
    global FILENAME
    FILENAME = filename

def read_robot_name():
    print "read_robot_name: "+FILENAME
    if os.path.isfile(FILENAME):
        print "FILENAME: "+FILENAME
        with open(FILENAME, 'r') as f:
            for line in f:
                print line
                if not (line.startswith('#') or len(line) == 0): 
                    name = line.rstrip() 
                    print "robot name: "+name
                    return name
    return ''

def write_robot_name(name):
    with open(FILENAME, 'w') as f:
        comment =  "# THIS IS A GENERATED FILE\n"
        comment += "# Here is the name the user gave to this robot\n"
        comment += "# This name does not affect anything,\n"
        comment += "# it is only useful for user to easily recognize the robot on desktop/mobile apps\n"
        f.write(comment)
        f.write(name)
