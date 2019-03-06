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
import RPi.GPIO as GPIO
import time
from threading import Lock

from gauss_msgs.msg import DigitalIOState
from gauss_msgs.srv import GetDigitalIO
from gauss_msgs.srv import SetDigitalIO

GPIO_1_A = 13
GPIO_1_B = 5
GPIO_1_C = 7 #pwm output
GPIO_2_A = 12
GPIO_2_B = 16
GPIO_2_C = 3
GPIO_2_D = 2

GPIO_PUMP = 26
GPIO_VALVE = 21

GPIO_1_A_NAME = '1A'
GPIO_1_B_NAME = '1B'
GPIO_1_C_NAME = '1C' #pwm output
GPIO_2_A_NAME = '2A'
GPIO_2_B_NAME = '2B'
GPIO_2_C_NAME = '2C'
GPIO_2_D_NAME = '2D'

GPIO_PUMP_NAME = 'PUMP'
GPIO_VALVE_NAME = 'VALVE'

class DigitalPin:

    def __init__(self, lock, pin, name, mode=GPIO.OUT, state=GPIO.LOW, pwm_flag=0, pwm_freq=50):
        self.lock = lock
        self.pin = pin
        self.name = name
        self.mode = mode
        self.pwm_flag = pwm_flag
        self.pwm_freq = pwm_freq
        self.set_mode(mode)
        rospy.sleep(0.01)
        if self.mode == GPIO.OUT:
            self.state = state
            GPIO.output(self.pin, self.state)
        else:
            self.state = GPIO.input(self.pin)

    def set_mode(self, mode):
        self.mode = mode
        if mode == GPIO.OUT and self.pwm_flag == 0:
            with self.lock:
                GPIO.setup(self.pin, GPIO.OUT)
            rospy.sleep(0.01)
            self.set_state(GPIO.LOW)
        elif mode == GPIO.OUT and self.pwm_flag == 1:
            with self.lock:
                GPIO.setup(self.pin, GPIO.OUT)
            rospy.sleep(0.01)
            GPIO.output(self.pin, GPIO.LOW)
            self.pwm = GPIO.PWM(self.pin, self.pwm_freq)
        else:
            with self.lock:
                GPIO.setup(self.pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

    def get_mode(self):
        return self.mode

    def set_state(self, state):
        if self.mode != GPIO.OUT:
            return False
        self.state = state
        with self.lock:
            if self.pwm_flag == 0:
                GPIO.output(self.pin, self.state)
            elif self.state == GPIO.HIGH and self.pwm_flag == 1:
                self.pwm_start_flag = 1
                self.pwm.start(5)
                for i in range(5, 71, 1):
                    self.pwm.ChangeDutyCycle(i)
                    time.sleep(.05)
            elif self.state == GPIO.LOW and self.pwm_flag == 1:
                if self.pwm_start_flag == 1:
                    self.pwm_start_flag = 0
                    for i in range(70, -1, -1):
                        self.pwm.ChangeDutyCycle(i)
                        time.sleep(.02)
                    self.pwm.stop()
                elif self.pwm_start_flag == 0:
                    self.pwm.stop()
        return True

    def get_state(self):
        if self.mode == GPIO.IN:
            with self.lock:
                self.state = GPIO.input(self.pin)
        return self.state

class DigitalIOPanel:

    def __init__(self):
        GPIO.setwarnings(False) 
        GPIO.setmode(GPIO.BCM)

        lock = Lock()

        self.publish_io_state_frequency = rospy.get_param("~publish_io_state_frequency")
        self.digitalIOs = [DigitalPin(lock, GPIO_1_A, GPIO_1_A_NAME), DigitalPin(lock, GPIO_1_B, GPIO_1_B_NAME), DigitalPin(lock, GPIO_1_C, GPIO_1_C_NAME, pwm_flag = 1, pwm_freq = 50),
                DigitalPin(lock, GPIO_2_A, GPIO_2_A_NAME), DigitalPin(lock, GPIO_2_B, GPIO_2_B_NAME), DigitalPin(lock, GPIO_2_C, GPIO_2_C_NAME),
		DigitalPin(lock, GPIO_2_D, GPIO_2_D_NAME), DigitalPin(lock, GPIO_PUMP, GPIO_PUMP_NAME), DigitalPin(lock, GPIO_VALVE, GPIO_VALVE_NAME)]

        self.digital_io_publisher = rospy.Publisher('gauss/rpi/digital_io_state', DigitalIOState, queue_size=1)
        rospy.Timer(rospy.Duration(1.0/self.publish_io_state_frequency), self.publish_io_state)

        self.get_io_server = rospy.Service('gauss/rpi/get_digital_io', GetDigitalIO, self.callback_get_io) 
        self.set_io_mode_server = rospy.Service('gauss/rpi/set_digital_io_mode', SetDigitalIO, self.callback_set_io_mode)
        self.set_io_state_server = rospy.Service('gauss/rpi/set_digital_io_state', SetDigitalIO, self.callback_set_io_state)

    def publish_io_state(self, event):
        msg = DigitalIOState()
        pins = []
        names = []
        modes = []
        states = []
        for io in self.digitalIOs:
            pins.append(io.pin)
            names.append(io.name)
            modes.append(io.mode)
            states.append(io.get_state())
        msg.pins = pins
        msg.names = names
        msg.modes = modes
        msg.states = states
        self.digital_io_publisher.publish(msg)

    def create_response(self, status, message):
        return {'status': status, 'message': message} 
 
    def callback_get_io(self, req):
        for io in self.digitalIOs:
            if io.pin == req.pin:
                return {
                        'status': 200,
                        'message': 'OK',
                        'pin': io.pin,
                        'name': io.name,
                        'mode': io.get_mode(),
                        'state': io.get_state()
                        }
        return self.create_response(400, "No GPIO found with this pin number (" + str(req.pin) + ")")

    def callback_set_io_mode(self, req):
        for io in self.digitalIOs:
            if io.pin == req.pin:
                # Set mode
                if req.value == 0:
                    io.set_mode(GPIO.OUT)
                else:
                    io.set_mode(GPIO.IN)
                return self.create_response(200, "Successfully set IO mode for PIN " + str(io.pin))
        # No pin found
        return self.create_response(400, "No GPIO found with this pin number (" + str(req.pin) + ")")

    def callback_set_io_state(self, req):
        for io in self.digitalIOs:
            if io.pin == req.pin:
                # Check gpio in in output mode
                if io.get_mode() != GPIO.OUT:
                    return self.create_response(400, "This PIN (" + str(io.pin) + ") is set as input, you can't change its state")
       
                # Set state
                success = False
                if req.value == 0:
                    success = io.set_state(GPIO.LOW)
                else:
                    success = io.set_state(GPIO.HIGH)
                
                if success:
                    return self.create_response(200, "Successfully set IO state for PIN " + str(io.pin))
                else:
                    return self.create_response(400, "Error : could not set IO state for PIN " + str(io.pin))
        
        # No pin found
        return self.create_response(400, "No GPIO found with this pin number (" + str(req.pin) + ")")

if __name__ == '__main__':
    rospy.init_node('digital_io_panel')
    DigitalIOPanel()
    rospy.spin()







































