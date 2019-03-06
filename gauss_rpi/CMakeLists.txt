cmake_minimum_required(VERSION 2.8.3)
project(gauss_rpi)

find_package(catkin REQUIRED COMPONENTS
  gauss_msgs
  rospy
  roscpp
  std_msgs
)

catkin_python_setup()

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)
