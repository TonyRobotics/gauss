#!/usr/bin/env python
# -*- coding: UTF-8 -*-

# 格式化成2016-03-20 11:45:39形式
# print time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()) 
import rospy
import subprocess
import os
import time 
from std_msgs.msg import String

class RosLogger:

    def __init__(self, debug_mode=False):
        self.debug_mode = debug_mode
        self.log_status_publisher = rospy.Publisher('/gauss/gaus_node/gauss_commander_log', String, queue_size=1)

    def publish_log_status(self, level, data):
        if self.debug_mode:
            msg = String()
            now_time = time.strftime("%Y-%m-%d %H:%M:%S ", time.localtime())
            msg.data = "gauss_commander " + level + " "+ now_time + data 
            self.log_status_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gauss_ros_logger')
    rl = RosLogger(debug_mode = True)
    while True:
        rl.publish_log_status("test")
    rospy.spin()

