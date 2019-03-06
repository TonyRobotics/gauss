^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_workbench_toolbox
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-07-20)
------------------
* upgraded read time #162
* added Dynamixel PRO information #162
* added readRegister function
* update dxl pro info
* update proInfo func
* modified max radian position
* Contributors: Darby Lim, Pyo, Taehun Lim

0.3.1 (2018-06-04)
------------------
* None

0.3.0 (2018-06-01)
------------------
* added getProtocolVersion()
* changed max_dxl_deries_num
* merged pull request `#152 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/152>`_ `#151 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/151>`_ `#149 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/149>`_ `#132 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/132>`_ 
* Contributors: Darby Lim, Pyo

0.2.4 (2018-03-20)
------------------
* changed package.xml to format v2
* Contributors: Pyo

0.2.3 (2018-03-09)
------------------
* None

0.2.2 (2018-02-28)
------------------
* modified the CI configurations (`#117 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/117>`_)
* modified the CMakeLists.txt to fix wrong path (`ros/rosdistro#17019 <https://github.com/ros/rosdistro/pull/17019>`_)
* Contributors: Pyo

0.2.1 (2018-02-22)
------------------
* modified the CMakeLists.txt to fix wrong path
* Contributors: Pyo

0.2.0 (2018-02-19)
------------------
* added conver function to PRO user
* added dxl_info_cnt init function
* added compatibility for different protocol
* added static
* added convert function
* added baudrate sorting
* added all dynamixel series
* added RX-10
* added millis
* added init dynamixel example
* added setting for packet handler
* added monitor example
* added item
* added dynamixel_item
* added toolbox_ros and modified arduino path
* added linux build and example
* added begin and getprotocolversion function
* modified linux version
* modified description
* modified model_info
* modified variable range
* modified setTools function
* modified sync function
* modified merge conflict
* modified variable name
* modified reset function
* modified function name and return variable name
* modified name of return var
* modified item name
* modified reset function
* modified item name (added underscore)
* modified function name
* modified function for ROS depend
* modified function to make space
* modified begin function to reduce storage space
* modified MX (2.0) protocol setting bug
* modified example
* modified sync and bulk comm
* modified lib
* modified begin
* modified variable
* modified begin function
* modified joint and wheel mode
* modified variable name
* modified begin function
* modified set function
* modified dynamixel item
* modified scan function
* modified folder tree
* modified dynamixel_tool
* modified toolbox structure
* modified .device and modified funtion for opencm and opencr
* modified ifdef
* modified get file
* modified arduino version
* modified get device in arduino
* fixed reset bug
* deleted dead code
* deleted empty space
* deleted xl define
* deleted debug code and update ping func
* test OpenCM
* Contributors: Darby Lim, Yoonseok Pyo

0.1.9 (2017-11-03)
------------------
* modified dependency
* Contributors: Darby Lim

0.1.8 (2017-11-01)
------------------
* None

0.1.7 (2017-10-30)
------------------
* added rospy for the issue https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/77
* Contributors: Darby Lim

0.1.6 (2017-08-09)
------------------
* bug fixed
* updated error msg
* updated get model path
* updated Dynamixel PRO
* updated Dynamixel XL, XM and XH
* updated annotation
* updated multi driver
* updated address name
* updated msg name
* modified launch files
* modified variable
* modified file location
* added sync read
* added multi read function
* added multi_driver
* changed BSD license to Apache 2.0 license
* Contributors: Darby Lim

0.1.5 (2017-05-23)
------------------
* modified the cmake of toolbox
* Contributors: Darby Lim

0.1.4 (2017-04-24)
------------------
* toolbox bug fixed
* added dynamixel new model: XL430_W250
* added dynamixel new model: XH
* renamed current controller -> torque controller
* Contributors: Darby Lim

0.1.3 (2016-11-29)
------------------
* modifiy folder path
* add drive_mode in XM series
* Contributors: Darby Lim

0.1.2 (2016-10-31)
------------------
* modify beta test feedback
* Contributors: Darby Lim

0.1.1 (2016-10-21)
------------------
* Revert "add baudrate combobox and modify velocity controller"
  This reverts commit f4f83761d687c40660a2c864aa4fcbebe1df4ea4.
* add baudrate combobox and modify velocity controller
* Contributors: Darby Lim

0.1.0 (2016-09-23)
-------------------------
* modified the package information for release
* edit cmake and xml files
* modify message
* add multiport controller and torque controller
* add position, velocity controller and pan-tilt, wheel tutorials
* add GUI package
* add pan tilt and wheel node in tutorial package
* add tutorial package
* add position, velocity, torque control package and change workbench_tool to workbench_toolbox
* Contributors: Darby Lim, Pyo
