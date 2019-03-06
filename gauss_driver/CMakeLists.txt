cmake_minimum_required(VERSION 2.8.3)
project(gauss_driver)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  hardware_interface
  controller_manager
  actionlib
  control_msgs
  geometry_msgs
  roscpp
  rospy
  std_msgs
  trajectory_msgs
  gauss_msgs
  mcp_can_rpi
  dynamixel_sdk
)
 
find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  CATKIN_DEPENDS 
    roscpp 
    rospy 
    std_msgs 
    hardware_interface 
    controller_manager 
    actionlib control_msgs 
    geometry_msgs 
    sensor_msgs 
    trajectory_msgs
    gauss_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gauss_driver
    src/change_hardware_version.cpp
    src/rpi/rpi_diagnostics.cpp
    src/ros_interface/ros_interface.cpp
    src/stepper/gauss_can_driver.cpp
    src/dynamixel/dxl_driver.cpp
    src/dynamixel/xl320_driver.cpp
    src/dynamixel/xl430_driver.cpp
    src/communication/dxl_communication.cpp
    src/communication/can_communication.cpp
    src/communication/gauss_communication.cpp
    src/communication/fake_communication.cpp
    src/hw_interface/gauss_hardware_interface.cpp
    src/gauss_driver_node.cpp
)

#
# wiringPi should be installed only on a Raspberry Pi board
#

EXECUTE_PROCESS( COMMAND uname -m COMMAND tr -d '\n' OUTPUT_VARIABLE ARCHITECTURE )
message( STATUS "Architecture: ${ARCHITECTURE}" )

if (${ARCHITECTURE} MATCHES "arm")
    message(STATUS "wiringPi library is required - arm processor")
    target_link_libraries(gauss_driver
        ${catkin_LIBRARIES} 
        -lwiringPi
    )
else()
    message(STATUS "wiringPi library not required")
    target_link_libraries(gauss_driver
        ${catkin_LIBRARIES} 
    )
endif()

add_dependencies(gauss_driver gauss_msgs_gencpp)
