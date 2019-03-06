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

import time
from threading import Thread
from socket import *
from robot_name_handler import *

#
# Will broadcast own IP address, so users can find the robot on a Wi-Fi network
#

def start_broadcast_ip_publisher():
    
    ##s.bind(('', 1664))
    robot_name = read_robot_name()

    while True:
        time.sleep(1)
        s = None
        try:
            s = socket(AF_INET, SOCK_DGRAM)
        except Exception: 
            continue
        # print "send udp " + robot_name
        try:
            s.setsockopt(SOL_SOCKET, SO_REUSEADDR, 1)
            s.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
            s.sendto(robot_name, ('255.255.255.255', 1665))
        except Exception:
            s.close()
            # print "Exception"
        #print "Send broadcast"
        s.close()

def udp_send():
    HOST = '<broadcast>'
    PORT = 1665
    BUFSIZE = 1024
    
    ADDR = (HOST, PORT)
    
    udpCliSock = socket(AF_INET, SOCK_DGRAM)
    udpCliSock.bind(('', 0))
    udpCliSock.setsockopt(SOL_SOCKET, SO_BROADCAST, 1)
    while True:
        data = "test"
        time.sleep(1)
        # print "send ok"
        # udpCliSock.sendto(bytes("data_test") ,ADDR)
        udpCliSock.sendto("device_test" ,ADDR)
    udpCliSock.close()

if __name__ == "__main__":
    start_broadcast_ip_publisher()
