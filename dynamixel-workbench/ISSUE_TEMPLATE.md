ISSUE TEMPLATE ver. 0.1.0

Before you open issue, please refer to [ROBOTIS e-Manual](http://emanual.robotis.com/)

1. Which distro of ROS have you installed? (ex, indigo, kinetic, melodic)


2. Which OS have you installed? (ex, Ubuntu 16.04 LTS (Xenial Xerus), Linux Mint 18.x, ...)


3. Which Dynamixel have you used? (ex, MX-28, XM430-W350, ...)

 
4. Write down the commands you used in order

  ```
  $ roscore    
  $ roslaunch dynamixel_workbench_single_manager single_manager.launch
  ```
 
5. Copy and Paste your error message on terminal

  ```
  SUMMARY
  ========

  PARAMETERS
   * /baud_rate: 57600
   * /device_name: /dev/ttyUSB0
   * /ping: False
   * /ping_id: 1
   * /rosdistro: kinetic
   * /rosversion: 1.12.13
   * /scan_range: 10

  NODES
    /
      single_dynamixel_controller (dynamixel_workbench_single_manager/single_dynamixel_controller)
      single_dynamixel_monitor (dynamixel_workbench_single_manager/single_dynamixel_monitor)

  auto-starting new master
  process[master]: started with pid [8594]
  ROS_MASTER_URI=http://192.168.1.3:11311

  setting /run_id to de3bd71e-728e-11e8-9f66-d8cb8af3d300
  process[rosout-1]: started with pid [8607]
  started core service [/rosout]
  process[single_dynamixel_monitor-2]: started with pid [8618]
  process[single_dynamixel_controller-3]: started with pid [8626]
  [PortHandlerLinux::SetupPort] Error opening serial port!
  ================================================================================REQUIRED process [single_dynamixel_monitor-2] has died!
  process has died [pid 8618, exit code -11, cmd /home/darby/catkin_ws/devel/lib/dynamixel_workbench_single_manager/single_dynamixel_monitor __name:=single_dynamixel_monitor __log:=/home/darby/.ros/log/de3bd71e-728e-11e8-9f66-d8cb8af3d300/single_dynamixel_monitor-2.log].
  log file: /home/darby/.ros/log/de3bd71e-728e-11e8-9f66-d8cb8af3d300/single_dynamixel_monitor-2*.log
  Initiating shutdown!
  ================================================================================
  [single_dynamixel_controller-3] killing on exit
  [single_dynamixel_monitor-2] killing on exit
  [rosout-1] killing on exit
  [master] killing on exit
  shutting down processing monitor...
  ... shutting down processing monitor complete
  ```
  
6. Please, describe detailedly what difficulty you are in 

    - HERE
