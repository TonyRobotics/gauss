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

    def __init__(self): 
        self.log_status_publisher = rospy.Publisher('/gauss/gaus_node/gauss_user_interface_log', String, queue_size=1)

    def publish_log_status(self, level, data):
        msg = String()
        now_time = time.strftime("%Y-%m-%d %H:%M:%S ", time.localtime())
        msg.data = "gauss_user_interface " + level + " "+ now_time + data 
        self.log_status_publisher.publish(msg)

if __name__ == '__main__':
    rospy.init_node('gauss_ros_logger')
    rl = RosLogger()
    while True:
        rl.publish_log_status("INFO", "test")
    rospy.spin()

