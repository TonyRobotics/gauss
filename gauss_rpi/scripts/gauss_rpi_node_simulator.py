#!/usr/bin/env python

import rospy
# import RPi.GPIO as GPIO

# from ros_log_manager import RosLogManager
# from led_manager import LEDManager
# from fans_manager import FansManager
# from gauss_button import GaussButton
# from digital_io_panel import DigitalIOPanel
# from wifi_connection import WifiConnectionManager
from gauss_ros_setup import *
#from gauss_modbus.modbus_server import ModbusServer
from shutdown_manager import ShutdownManager

class GaussRpi:

    def __init__(self):
        self.wifi_manager_enabled = rospy.get_param("~wifi_manager_enabled")
        self.launch_ros_processes = rospy.get_param("~launch_ros_processes")
        #self.modbus_server_enabled = rospy.get_param("~modbus_server_enabled")
        #self.modbus_server_address = rospy.get_param("~modbus_server_address")
        #self.modbus_server_port = rospy.get_param("~modbus_server_port")
        self.gauss_ros_setup = None
    
        if self.launch_ros_processes:
            self.gauss_ros_setup = GaussRosSetup()
            rospy.sleep(10) # let some time for other processes to launch (does not affect total launch time)
        
        # Start wifi manager
        # if self.wifi_manager_enabled:
            # self.wifi_manager = WifiConnectionManager()
      
        # self.fans_manager = FansManager()
        # self.ros_log_manager = RosLogManager()
        self.shutdown_manager = ShutdownManager()
        # self.led_manager = LEDManager()
        # self.gauss_button = GaussButton()
        # self.digital_io_panel = DigitalIOPanel()

        # Start Modbus server
        #if self.modbus_server_enabled:
        #    self.modbus_server = ModbusServer(self.modbus_server_address, self.modbus_server_port)
        #    rospy.on_shutdown(self.modbus_server.stop)
        #    self.modbus_server.start()
        
         
                        

if __name__ == '__main__':
    rospy.init_node('gauss_rpi')
    GaussRpi()
    rospy.spin()
