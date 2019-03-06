^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package dynamixel_workbench_single_manager_gui
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

1.0.0 (2018-07-20)
------------------
* upgraded read time #162
* added Dynamixel PRO information #162
* added mutex for log thread
* added timer for data log
* added PROext header
* updated dxl ext function
* Contributors: Darby Lim, Pyo

0.3.1 (2018-06-04)
------------------
* updated Qt5 and delete Qt4 config
* deleted rqt_plugin header
* merged pull request `#154 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/154>`_ `#153 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/153>`_
* Contributors: Darby Lim

0.3.0 (2018-06-01)
------------------
* changed compile options for qt5
* merged pull request `#152 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/152>`_ `#151 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/151>`_ `#149 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/149>`_ `#132 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/132>`_ 
* Contributors: Darby Lim, Pyo

0.2.4 (2018-03-20)
------------------
* changed package.xml to format v2
* Contributors: Pyo

0.2.3 (2018-03-09)
------------------
* modified dialog rotate direction
* added dynamixel_sdk lib
* Contributors: Darby Lim

0.2.2 (2018-02-28)
------------------
* modified the CI configurations (`#117 <https://github.com/ROBOTIS-GIT/dynamixel-workbench/issues/117>`_)
* modified the CMakeLists.txt to fix wrong path (`ros/rosdistro#17019 <https://github.com/ros/rosdistro/pull/17019>`_)
* Contributors: Pyo

0.2.1 (2018-02-22)
------------------
* None

0.2.0 (2018-02-19)
------------------
* added baudrate sorting
* added baud rate item and operating mode item
* added linux build and example
* modified dynamixel workbench lib
* modified make file
* modified function
* deleted build
* Contributors: Darby Lim

0.1.9 (2017-11-03)
------------------
* deleted libqt4
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
* added dynamixel new model: XL430_W250
* added dynamixel new model: XH
* Contributors: Darby Lim

0.1.3 (2016-11-29)
------------------
* add drive_mode in XM series
* update single manager and GUI
* modified msgs files
* Contributors: Darby Lim

0.1.2 (2016-10-31)
------------------
* add comment in msgs file
* add stop sign in velocity controller
* modify beta test feedback
* Contributors: Darby Lim

0.1.1 (2016-10-21)
------------------
* modified single manager and gui
* modified factory reset
* modified baudrate, factory reset, reboot and velocity controller
* Revert "add baudrate combobox and modify velocity controller"
  This reverts commit f4f83761d687c40660a2c864aa4fcbebe1df4ea4.
* add baudrate combobox and modify velocity controller
* Contributors: Darby Lim

0.1.0 (2016-09-23)
-------------------------
* modified the package information for release
* edit cmake and xml files
* edit GUI initialization
* edit launch file
* add multiport controller and torque controller
* add position, velocity controller and pan-tilt, wheel tutorials
* add gui package and modify position controller
* Contributors: Darby Lim, pyo
