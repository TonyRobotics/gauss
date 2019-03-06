################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_workbench_msgs)

################################################################################
# Find catkin packages and libraries for catkin and system dependencies
################################################################################
find_package(catkin REQUIRED COMPONENTS
  std_msgs
  message_generation
)

################################################################################
# Setup for python modules and scripts
################################################################################

################################################################################
# Declare ROS messages, services and actions
################################################################################
add_message_files(
  FILES
  AX.msg
  EX.msg
  MX.msg
  MX2.msg
  MX2Ext.msg
  MXExt.msg
  PRO.msg
  PROExt.msg
  RX.msg
  XH.msg
  XL.msg
  XL320.msg
  XM.msg
  XMExt.msg
  DynamixelState.msg
  DynamixelStateList.msg
  DynamixelInfo.msg
  DynamixelLoadInfo.msg
)

add_service_files(
  FILES
  JointCommand.srv
  WheelCommand.srv
  GetDynamixelInfo.srv
  DynamixelCommand.srv
)

generate_messages(
  DEPENDENCIES std_msgs
)

################################################################################
# Declare ROS dynamic reconfigure parameters
################################################################################

################################################################################
# Declare catkin specific configuration to be passed to dependent projects
################################################################################
catkin_package(
   CATKIN_DEPENDS std_msgs message_runtime
)

################################################################################
# Build
################################################################################

################################################################################
# Install
################################################################################

################################################################################
# Test
################################################################################
