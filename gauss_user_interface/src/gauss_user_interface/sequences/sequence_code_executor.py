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
import os
import subprocess

from std_srvs.srv import SetBool

def create_directory(directory_path):
    if not os.path.exists(directory_path):
        os.makedirs(directory_path)

def save_to_file(filename, text):
    with open(filename, 'w') as target:
        target.write(text)

def read_file(filename):
    with open(filename, 'r') as target:
	return target.read()	

class SequenceCodeExecutor:

    def __init__(self):
        self.blockly_dir = rospy.get_param("~sequence_code_to_execute_path")
        self.python_file = str(self.blockly_dir) + '/generated_code.py'
        create_directory(self.blockly_dir)
        self.process = None
        self.cancel_flag = False

    def execute_generated_code(self, python_code): 
        # 1. Check if still executing
        if self.is_executing_code():
            return { 'status': 400, 'message': "A generated code is already running" }

        # 2. Save code in python file
        save_to_file(self.python_file, python_code)

        # 3. Start executing code with process
        self.process = subprocess.Popen(['python', str(self.python_file)],
                stdout=subprocess.PIPE, stderr=subprocess.PIPE)

        # 4. Wait for process to finish
        output = self.process.communicate()

        # 5. Check if cmd was canceled
        if self.cancel_flag:
            self.cancel_flag = False
            self.stop_robot_action()
            return { 'status': 300, 'message': 'Execution of Sequence has been canceled' }

        # 6. Check error in output
        error = output[1]

        if error != '':
            str_to_find = 'gauss_python_api.gauss_api.GaussException:'
            exception_index = error.find(str_to_find)

            # found GaussException -> Gauss error
            if exception_index != -1:
                exception_index = exception_index + len(str_to_find) + 1
                return { 'status': 400, 'message': error[exception_index:].rstrip() }
            # Any other exception 
            else:
                return { 'status': 400, 'message': 'Error while executing generated code : ' + str(error)}

        # 7. Return OK if no error
        return { 'status': 200, 'message': 'Successfully executed Sequence code' }

    def cancel_execution(self):
        if self.process is None:
            return
        if self.is_executing_code():
            rospy.logwarn("Stopping sequence code execution")
            self.cancel_flag = True
            self.process.terminate()
    
    def stop_robot_action(self):
        # Stop current move command
        try:
            rospy.wait_for_service('/gauss/commander/stop_command', 1)
            stop_cmd = rospy.ServiceProxy('/gauss/commander/stop_command', SetBool)
            stop_cmd()
        except (rospy.ServiceException, rospy.ROSException), e:
            pass

    def is_executing_code(self):
        if self.process is None:
            return False
        return_code = self.process.poll()
        return (return_code is None)

