/*******************************************************************************
* Copyright 2016 ROBOTIS CO., LTD.
*
* Licensed under the Apache License, Version 2.0 (the "License");
* you may not use this file except in compliance with the License.
* You may obtain a copy of the License at
*
*     http://www.apache.org/licenses/LICENSE-2.0
*
* Unless required by applicable law or agreed to in writing, software
* distributed under the License is distributed on an "AS IS" BASIS,
* WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
* See the License for the specific language governing permissions and
* limitations under the License.
*******************************************************************************/

/* Authors: Taehun Lim (Darby) */

#include <ros/ros.h>
#include <ros/network.h>

#include <string>
#include <sstream>
#include <std_msgs/String.h>
#include "../include/dynamixel_workbench_single_manager_gui/qnode.hpp"

using namespace qnode;

QNode::QNode(int argc, char** argv )
    :init_argc(argc),
     init_argv(argv),
     row_count_(0)
{}

QNode::~QNode()
{
  if(ros::isStarted())
  {
    ros::shutdown();
    ros::waitForShutdown();
  }
  wait();
}

bool QNode::init()
{
  ros::init(init_argc,init_argv,"single_manager_gui");

  if (!ros::master::check())
      return false;

  ros::start();
  ros::NodeHandle node_handle;

  // Init Service Client
  dynamixel_info_client_    = node_handle.serviceClient<dynamixel_workbench_msgs::GetDynamixelInfo>("dynamixel/info");
  dynamixel_command_client_ = node_handle.serviceClient<dynamixel_workbench_msgs::DynamixelCommand>("dynamixel/command");
  getDynamixelInfo();

  // Init Message Subscriber
  initDynamixelStateSubscriber();

  // Init mutex
  mutex = PTHREAD_MUTEX_INITIALIZER;

  start();
  return true;
}

bool QNode::sendCommandMsg(std::string cmd, std::string addr, int64_t value)
{
  dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

  set_dynamixel_command.request.command   = cmd;
  set_dynamixel_command.request.addr_name = addr;
  set_dynamixel_command.request.value     = value;

  if (dynamixel_command_client_.call(set_dynamixel_command))
  {
    if (!set_dynamixel_command.response.comm_result)
      return false;
    else
      return true;
  }
}

bool QNode::sendSetIdMsg(uint8_t set_id)
{
  if (sendCommandMsg("addr", "ID", set_id))
    return true;
  else
    return false;
}

bool QNode::sendSetBaudrateMsg(int64_t baud_rate)
{
  if (sendCommandMsg("addr", "Baud_Rate", baud_rate))
    return true;
  else
    return false;
}

bool QNode::sendSetOperatingModeMsg(std::string index, float protocol_version, std::string model_name, int32_t value_of_max_radian_position)
{
  dynamixel_workbench_msgs::DynamixelCommand set_dynamixel_command;

  if (protocol_version == 1.0)
  {
    if (index == "position_control")
    {
      if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", value_of_max_radian_position))
        return true;
      else
        return false;
    }
    else if (index == "velocity_control")
    {
      if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", 0))
        return true;
      else
        return false;
    }
    else if (index == "extended_position_control")
    {
      if (sendCommandMsg("addr", "CW_Angle_Limit", value_of_max_radian_position) && sendCommandMsg("addr", "CCW_Angle_Limit", value_of_max_radian_position))
        return true;
      else
        return false;
    }
  }
  else
  {
    if (model_name.find("MX") != std::string::npos)
    {
      if (model_name.find("MX-28-2") != std::string::npos)
      {
        if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 1))
            return true;
          else
            return false;
        }
        else if (index == "position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 3))
            return true;
          else
            return false;
        }
        else if (index == "extended_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 4))
            return true;
          else
            return false;
        }
        else if (index == "pwm_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 16))
            return true;
          else
            return false;
        }
      }
      else if (model_name.find("MX-64-2") != std::string::npos ||
               model_name.find("MX-106-2") != std::string::npos )
      {
        if (index == "current_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 0))
            return true;
          else
            return false;
        }
        else if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 1))
            return true;
          else
            return false;
        }
        else if (index == "position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 3))
            return true;
          else
            return false;
        }
        else if (index == "extended_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 4))
            return true;
          else
            return false;
        }
        else if (index == "current_based_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 5))
            return true;
          else
            return false;
        }
        else if (index == "pwm_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 16))
            return true;
          else
            return false;
        }
      }
    }
    if (model_name.find("XL") != std::string::npos)
    {
      if (model_name.find("XL-320") != std::string::npos)
      {
        if (index == "position_control")
        {
          if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", value_of_max_radian_position))
            return true;
          else
            return false;
        }
        else if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "CW_Angle_Limit", 0) && sendCommandMsg("addr", "CCW_Angle_Limit", 0))
            return true;
          else
            return false;
        }
      }
      else if (model_name.find("XL430-W250") != std::string::npos)
      {
        if (index == "velocity_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 1))
            return true;
          else
            return false;
        }
        else if (index == "position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 3))
            return true;
          else
            return false;
        }
        else if (index == "extended_position_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 4))
            return true;
          else
            return false;
        }
        else if (index == "pwm_control")
        {
          if (sendCommandMsg("addr", "Operating_Mode", 16))
            return true;
          else
            return false;
        }
      }
    }
    if (model_name.find("XM") != std::string::npos  ||
        model_name.find("XH") != std::string::npos   )
    {
      if (index == "current_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 0))
          return true;
        else
          return false;
      }
      else if (index == "velocity_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 1))
          return true;
        else
          return false;
      }
      else if (index == "position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 3))
          return true;
        else
          return false;
      }
      else if (index == "extended_position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 4))
          return true;
        else
          return false;
      }
      else if (index == "current_based_position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 5))
          return true;
        else
          return false;
      }
      else if (index == "pwm_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 16))
          return true;
        else
          return false;
      }
    }
    else if (model_name.find("PRO") != std::string::npos)
    {
      if (index == "torque_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 0))
          return true;
        else
          return false;
      }
      else if (index == "velocity_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 1))
          return true;
        else
          return false;
      }
      else if (index == "position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 3))
          return true;
        else
          return false;
      }
      else if (index == "extended_position_control")
      {
        if (sendCommandMsg("addr", "Operating_Mode", 4))
          return true;
        else
          return false;
      }
    }
  }
}

bool QNode::sendTorqueMsg(int64_t onoff)
{
  if (sendCommandMsg("addr", "Torque_Enable", onoff))
    return true;
  else
    return false;
}

bool QNode::sendRebootMsg(void)
{
  if (sendCommandMsg("reboot"))
    return true;
  else
    return false;
}

bool QNode::sendResetMsg(void)
{
  if (sendCommandMsg("factory_reset"))
  {
    getDynamixelInfo();
    return true;
  }
  else
  {
    return false;
  }
}

bool QNode::setPositionZeroMsg(int32_t zero_position)
{
  if (sendCommandMsg("addr", "Goal_Position", zero_position))
    return true;
  else
    return false;
}

bool QNode::sendAddressValueMsg(std::string addr_name, int64_t value)
{
  if (sendCommandMsg("addr", addr_name, value))
    return true;
  else
    return false;
}

void QNode::getDynamixelInfo()
{
  dynamixel_workbench_msgs::GetDynamixelInfo get_dynamixel_info;

  if (dynamixel_info_client_.call(get_dynamixel_info))
  {
    dynamixel_info_.load_info.device_name      = get_dynamixel_info.response.dynamixel_info.load_info.device_name;
    dynamixel_info_.load_info.baud_rate        = get_dynamixel_info.response.dynamixel_info.load_info.baud_rate;
    dynamixel_info_.load_info.protocol_version = get_dynamixel_info.response.dynamixel_info.load_info.protocol_version;

    dynamixel_info_.model_id         = get_dynamixel_info.response.dynamixel_info.model_id;
    dynamixel_info_.model_name       = get_dynamixel_info.response.dynamixel_info.model_name;
    dynamixel_info_.model_number     = get_dynamixel_info.response.dynamixel_info.model_number;

    Q_EMIT updateDynamixelInfo(dynamixel_info_);
  }
}

void QNode::initDynamixelStateSubscriber()
{
  ros::NodeHandle node_handle;

  if (dynamixel_info_.model_name.find("AX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("AX"), 10, &QNode::AXStatusMsgCallback, this);
    dynamixelDataLogPtr = &QNode::AX;
  }
  else if (dynamixel_info_.model_name.find("RX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("RX"), 10, &QNode::RXStatusMsgCallback, this);
    dynamixelDataLogPtr = &QNode::RX;
  }
  else if (dynamixel_info_.model_name.find("MX") != std::string::npos)
  {
    if (dynamixel_info_.model_name.find("MX-28-2") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MX2StatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::MX2;
    }
    else if (dynamixel_info_.model_name.find("MX-64-2") != std::string::npos ||
             dynamixel_info_.model_name.find("MX-106-2") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MX2ExtStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::MX2Ext;
    }
    else if (dynamixel_info_.model_name.find("MX-12W") != std::string::npos ||
        dynamixel_info_.model_name.find("MX-28") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MXStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::MX;
    }
    else if (dynamixel_info_.model_name.find("MX-64") != std::string::npos ||
             dynamixel_info_.model_name.find("MX-106") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("MX"), 10, &QNode::MXExtStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::MXExt;
    }
  }
  else if (dynamixel_info_.model_name.find("EX") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("EX"), 10, &QNode::EXStatusMsgCallback, this);
    dynamixelDataLogPtr = &QNode::EX;
  }
  else if (dynamixel_info_.model_name.find("XL") != std::string::npos)
  {
    if (dynamixel_info_.model_name.find("XL-320") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XL"), 10, &QNode::XL320StatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::XL320;
    }
    else if (dynamixel_info_.model_name.find("XL430-W250") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XL"), 10, &QNode::XLStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::XL;
    }
  }
  else if (dynamixel_info_.model_name.find("XM") != std::string::npos)
  {
    if (dynamixel_info_.model_name.find("XM430-W210") != std::string::npos ||
        dynamixel_info_.model_name.find("XM430-W350") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XM"), 10, &QNode::XMStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::XM;
    }
    else if (dynamixel_info_.model_name.find("XM540-W150") != std::string::npos ||
             dynamixel_info_.model_name.find("XM540-W270") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XM"), 10, &QNode::XMExtStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::XMExt;
    }
  }
  else if (dynamixel_info_.model_name.find("XH") != std::string::npos)
  {
    dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("XH"), 10, &QNode::XHStatusMsgCallback, this);
    dynamixelDataLogPtr = &QNode::XH;
  }
  else if (dynamixel_info_.model_name.find("PRO") != std::string::npos)
  {
    if (dynamixel_info_.model_name.find("PRO_L42_10_S300_R") != std::string::npos)
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("PRO"), 10, &QNode::PROStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::PRO;
    }
    else
    {
      dynamixel_status_msg_sub_ = node_handle.subscribe("dynamixel/" + std::string("PRO"), 10, &QNode::PROExtStatusMsgCallback, this);
      dynamixelDataLogPtr = &QNode::PROExt;
    }
  }
}

void QNode::run()
{
  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    ros::spinOnce();
    loop_rate.sleep();
  }

  std::cout << "ROS shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT rosShutdown();
}

void QNode::log(const std::string &msg, int64_t sender)
{
  if(logging_model_.rowCount() == row_count_)
      logging_model_.insertRows(row_count_, 1);

  std::stringstream logging_model_msg;

  logging_model_msg << msg << sender;

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(row_count_), new_row);

  row_count_++;
}

void QNode::log(const std::string &msg)
{
  if(logging_model_.rowCount() == row_count_)
      logging_model_.insertRows(row_count_, 1);

  std::stringstream logging_model_msg;

  logging_model_msg << msg;

  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model_.setData(logging_model_.index(row_count_), new_row);

  row_count_++;
}

void QNode::writeReceivedDynamixelData()
{
  pthread_mutex_lock(&mutex);
  (this->*dynamixelDataLogPtr)();
  pthread_mutex_unlock(&mutex);
  row_count_ = 0;
}

void QNode::AX()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), ax_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), ax_msgs_.Firmware_Version);
  log(std::string("ID: "), ax_msgs_.ID);
  log(std::string("Baud_Rate: "), ax_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), ax_msgs_.Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), ax_msgs_.CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), ax_msgs_.CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), ax_msgs_.Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), ax_msgs_.Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), ax_msgs_.Max_Voltage_Limit);
  log(std::string("Max_Torque: "), ax_msgs_.Max_Torque);
  log(std::string("Status_Return_Level: "), ax_msgs_.Status_Return_Level);
  log(std::string("Alarm_LED: "), ax_msgs_.Alarm_LED);
  log(std::string("Shutdown: "), ax_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), ax_msgs_.Torque_Enable);
  log(std::string("LED: "), ax_msgs_.LED);
  log(std::string("CW_Compliance_Margin: "), ax_msgs_.CW_Compliance_Margin);
  log(std::string("CCW_Compliance_Margin: "), ax_msgs_.CCW_Compliance_Margin);
  log(std::string("CW_Compliance_Slope: "), ax_msgs_.CW_Compliance_Slope);
  log(std::string("CCW_Compliance_Slope: "), ax_msgs_.CCW_Compliance_Slope);
  log(std::string("Goal_Position: "), ax_msgs_.Goal_Position);
  log(std::string("Moving_Speed: "), ax_msgs_.Moving_Speed);
  log(std::string("Torque_Limit: "), ax_msgs_.Torque_Limit);
  log(std::string("Present_Position: "), ax_msgs_.Present_Position);
  log(std::string("Present_Speed: "), ax_msgs_.Present_Speed);
  log(std::string("Present_Load: "), ax_msgs_.Present_Load);
  log(std::string("Present_Voltage: "), ax_msgs_.Present_Voltage);
  log(std::string("Present_Temperature: "), ax_msgs_.Present_Temperature);
  log(std::string("Registered: "), ax_msgs_.Registered);
  log(std::string("Moving: "), ax_msgs_.Moving);
  log(std::string("Lock: "), ax_msgs_.Lock);
  log(std::string("Punch: "), ax_msgs_.Punch);
}

void QNode::RX()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), rx_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), rx_msgs_.Firmware_Version);
  log(std::string("ID: "), rx_msgs_.ID);
  log(std::string("Baud_Rate: "), rx_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), rx_msgs_.Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), rx_msgs_.CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), rx_msgs_.CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), rx_msgs_.Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), rx_msgs_.Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), rx_msgs_.Max_Voltage_Limit);
  log(std::string("Max_Torque: "), rx_msgs_.Max_Torque);
  log(std::string("Status_Return_Level: "), rx_msgs_.Status_Return_Level);
  log(std::string("Alarm_LED: "), rx_msgs_.Alarm_LED);
  log(std::string("Shutdown: "), rx_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), rx_msgs_.Torque_Enable);
  log(std::string("LED: "), rx_msgs_.LED);
  log(std::string("CW_Compliance_Margin: "), rx_msgs_.CW_Compliance_Margin);
  log(std::string("CCW_Compliance_Margin: "), rx_msgs_.CCW_Compliance_Margin);
  log(std::string("CW_Compliance_Slope: "), rx_msgs_.CW_Compliance_Slope);
  log(std::string("CCW_Compliance_Slope: "), rx_msgs_.CCW_Compliance_Slope);
  log(std::string("Goal_Position: "), rx_msgs_.Goal_Position);
  log(std::string("Moving_Speed: "), rx_msgs_.Moving_Speed);
  log(std::string("Torque_Limit: "), rx_msgs_.Torque_Limit);
  log(std::string("Present_Position: "), rx_msgs_.Present_Position);
  log(std::string("Present_Speed: "), rx_msgs_.Present_Speed);
  log(std::string("Present_Load: "), rx_msgs_.Present_Load);
  log(std::string("Present_Voltage: "), rx_msgs_.Present_Voltage);
  log(std::string("Present_Temperature: "), rx_msgs_.Present_Temperature);
  log(std::string("Registered: "), rx_msgs_.Registered);
  log(std::string("Moving: "), rx_msgs_.Moving);
  log(std::string("Lock: "), rx_msgs_.Lock);
  log(std::string("Punch: "), rx_msgs_.Punch);
}

void QNode::MX()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), mx_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), mx_msgs_.Firmware_Version);
  log(std::string("ID: "), mx_msgs_.ID);
  log(std::string("Baud_Rate: "), mx_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), mx_msgs_.Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), mx_msgs_.CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), mx_msgs_.CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), mx_msgs_.Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), mx_msgs_.Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), mx_msgs_.Max_Voltage_Limit);
  log(std::string("Max_Torque: "), mx_msgs_.Max_Torque);
  log(std::string("Status_Return_Level: "), mx_msgs_.Status_Return_Level);
  log(std::string("Alarm_LED: "), mx_msgs_.Alarm_LED);
  log(std::string("Shutdown: "), mx_msgs_.Shutdown);
  log(std::string("Multi_Turn_Offset: "), mx_msgs_.Multi_Turn_Offset);
  log(std::string("Resolution_Divider: "), mx_msgs_.Resolution_Divider);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), mx_msgs_.Torque_Enable);
  log(std::string("LED: "), mx_msgs_.LED);
  log(std::string("D_gain: "), mx_msgs_.D_gain);
  log(std::string("I_gain: "), mx_msgs_.I_gain);
  log(std::string("P_gain: "), mx_msgs_.P_gain);
  log(std::string("Goal_Position: "), mx_msgs_.Goal_Position);
  log(std::string("Moving_Speed: "), mx_msgs_.Moving_Speed);
  log(std::string("Torque_Limit: "), mx_msgs_.Torque_Limit);
  log(std::string("Present_Position: "), mx_msgs_.Present_Position);
  log(std::string("Present_Speed: "), mx_msgs_.Present_Speed);
  log(std::string("Present_Load: "), mx_msgs_.Present_Load);
  log(std::string("Present_Voltage: "), mx_msgs_.Present_Voltage);
  log(std::string("Present_Temperature: "), mx_msgs_.Present_Temperature);
  log(std::string("Registered: "), mx_msgs_.Registered);
  log(std::string("Moving: "), mx_msgs_.Moving);
  log(std::string("Lock: "), mx_msgs_.Lock);
  log(std::string("Punch: "), mx_msgs_.Punch);
  log(std::string("Goal_Acceleration: "), mx_msgs_.Goal_Acceleration);
}

void QNode::MXExt()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), mxext_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), mxext_msgs_.Firmware_Version);
  log(std::string("ID: "), mxext_msgs_.ID);
  log(std::string("Baud_Rate: "), mxext_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), mxext_msgs_.Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), mxext_msgs_.CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), mxext_msgs_.CCW_Angle_Limit);
  log(std::string("Temperature_Limit: "), mxext_msgs_.Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), mxext_msgs_.Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), mxext_msgs_.Max_Voltage_Limit);
  log(std::string("Max_Torque: "), mxext_msgs_.Max_Torque);
  log(std::string("Status_Return_Level: "), mxext_msgs_.Status_Return_Level);
  log(std::string("Alarm_LED: "), mxext_msgs_.Alarm_LED);
  log(std::string("Shutdown: "), mxext_msgs_.Shutdown);
  log(std::string("Multi_Turn_Offset: "), mxext_msgs_.Multi_Turn_Offset);
  log(std::string("Resolution_Divider: "), mxext_msgs_.Resolution_Divider);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), mxext_msgs_.Torque_Enable);
  log(std::string("LED: "), mxext_msgs_.LED);
  log(std::string("D_gain: "), mxext_msgs_.D_gain);
  log(std::string("I_gain: "), mxext_msgs_.I_gain);
  log(std::string("P_gain: "), mxext_msgs_.P_gain);
  log(std::string("Goal_Position: "), mxext_msgs_.Goal_Position);
  log(std::string("Moving_Speed: "), mxext_msgs_.Moving_Speed);
  log(std::string("Torque_Limit: "), mxext_msgs_.Torque_Limit);
  log(std::string("Present_Position: "), mxext_msgs_.Present_Position);
  log(std::string("Present_Speed: "), mxext_msgs_.Present_Speed);
  log(std::string("Present_Load: "), mxext_msgs_.Present_Load);
  log(std::string("Present_Voltage: "), mxext_msgs_.Present_Voltage);
  log(std::string("Present_Temperature: "), mxext_msgs_.Present_Temperature);
  log(std::string("Registered: "), mxext_msgs_.Registered);
  log(std::string("Moving: "), mxext_msgs_.Moving);
  log(std::string("Lock: "), mxext_msgs_.Lock);
  log(std::string("Punch: "), mxext_msgs_.Punch);
  log(std::string("Current: "), mxext_msgs_.Current);
  log(std::string("Torque_Control_Mode_Enable: "), mxext_msgs_.Torque_Control_Mode_Enable);
  log(std::string("Goal_Torque: "), mxext_msgs_.Goal_Torque);
  log(std::string("Goal_Acceleration: "), mxext_msgs_.Goal_Acceleration);
}

void QNode::MX2()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), mx2_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), mx2_msgs_.Firmware_Version);
  log(std::string("ID: "), mx2_msgs_.ID);
  log(std::string("Baud_Rate: "), mx2_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), mx2_msgs_.Return_Delay_Time);
  log(std::string("Drive_Mode: "), mx2_msgs_.Drive_Mode);
  log(std::string("Operating_Mode: "), mx2_msgs_.Operating_Mode);
  log(std::string("Secondary_ID: "), mx2_msgs_.Secondary_ID);
  log(std::string("Protocol_Version: "), mx2_msgs_.Protocol_Version);
  log(std::string("Homing_Offset: "), mx2_msgs_.Homing_Offset);
  log(std::string("Moving_Threshold: "), mx2_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), mx2_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), mx2_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), mx2_msgs_.Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), mx2_msgs_.PWM_Limit);
  log(std::string("Acceleration_Limit: "), mx2_msgs_.Acceleration_Limit);
  log(std::string("Velocity_Limit: "), mx2_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), mx2_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), mx2_msgs_.Min_Position_Limit);
  log(std::string("Shutdown: "), mx2_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), mx2_msgs_.Torque_Enable);
  log(std::string("LED: "), mx2_msgs_.LED);
  log(std::string("Status_Return_Level: "), mx2_msgs_.Status_Return_Level);
  log(std::string("Registered_Instruction: "), mx2_msgs_.Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), mx2_msgs_.Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), mx2_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), mx2_msgs_.Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), mx2_msgs_.Position_D_Gain);
  log(std::string("Position_I_Gain: "), mx2_msgs_.Position_I_Gain);
  log(std::string("Position_P_Gain: "), mx2_msgs_.Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), mx2_msgs_.Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), mx2_msgs_.Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), mx2_msgs_.Bus_Watchdog);
  log(std::string("Goal_PWM: "), mx2_msgs_.Goal_PWM);
  log(std::string("Goal_Velocity: "), mx2_msgs_.Goal_Velocity);
  log(std::string("Profile_Acceleration: "), mx2_msgs_.Profile_Acceleration);
  log(std::string("Profile_Velocity: "), mx2_msgs_.Profile_Velocity);
  log(std::string("Goal_Position: "), mx2_msgs_.Goal_Position);
  log(std::string("Realtime_Tick: "), mx2_msgs_.Realtime_Tick);
  log(std::string("Moving: "), mx2_msgs_.Moving);
  log(std::string("Moving_Status: "), mx2_msgs_.Moving_Status);
  log(std::string("Present_PWM: "), mx2_msgs_.Present_PWM);
  log(std::string("Present_Load: "), mx2_msgs_.Present_Load);
  log(std::string("Present_Velocity: "), mx2_msgs_.Present_Velocity);
  log(std::string("Present_Position: "), mx2_msgs_.Present_Position);
  log(std::string("Velocity_Trajectory: "), mx2_msgs_.Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), mx2_msgs_.Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), mx2_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), mx2_msgs_.Present_Temperature);
}

void QNode::MX2Ext()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), mx2ext_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), mx2ext_msgs_.Firmware_Version);
  log(std::string("ID: "), mx2ext_msgs_.ID);
  log(std::string("Baud_Rate: "), mx2ext_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), mx2ext_msgs_.Return_Delay_Time);
  log(std::string("Drive_Mode: "), mx2ext_msgs_.Drive_Mode);
  log(std::string("Operating_Mode: "), mx2ext_msgs_.Operating_Mode);
  log(std::string("Secondary_ID: "), mx2ext_msgs_.Secondary_ID);
  log(std::string("Protocol_Version: "), mx2ext_msgs_.Protocol_Version);
  log(std::string("Homing_Offset: "), mx2ext_msgs_.Homing_Offset);
  log(std::string("Moving_Threshold: "), mx2ext_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), mx2ext_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), mx2ext_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), mx2ext_msgs_.Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), mx2ext_msgs_.PWM_Limit);
  log(std::string("Current_Limit: "), mx2ext_msgs_.Current_Limit);
  log(std::string("Acceleration_Limit: "), mx2ext_msgs_.Acceleration_Limit);
  log(std::string("Velocity_Limit: "), mx2ext_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), mx2ext_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), mx2ext_msgs_.Min_Position_Limit);
  log(std::string("Shutdown: "), mx2ext_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), mx2ext_msgs_.Torque_Enable);
  log(std::string("LED: "), mx2ext_msgs_.LED);
  log(std::string("Status_Return_Level: "), mx2ext_msgs_.Status_Return_Level);
  log(std::string("Registered_Instruction: "), mx2ext_msgs_.Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), mx2ext_msgs_.Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), mx2ext_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), mx2ext_msgs_.Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), mx2ext_msgs_.Position_D_Gain);
  log(std::string("Position_I_Gain: "), mx2ext_msgs_.Position_I_Gain);
  log(std::string("Position_P_Gain: "), mx2ext_msgs_.Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), mx2ext_msgs_.Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), mx2ext_msgs_.Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), mx2ext_msgs_.Bus_Watchdog);
  log(std::string("Goal_PWM: "), mx2ext_msgs_.Goal_PWM);
  log(std::string("Goal_Velocity: "), mx2ext_msgs_.Goal_Velocity);
  log(std::string("Profile_Acceleration: "), mx2ext_msgs_.Profile_Acceleration);
  log(std::string("Profile_Velocity: "), mx2ext_msgs_.Profile_Velocity);
  log(std::string("Goal_Position: "), mx2ext_msgs_.Goal_Position);
  log(std::string("Realtime_Tick: "), mx2ext_msgs_.Realtime_Tick);
  log(std::string("Moving: "), mx2ext_msgs_.Moving);
  log(std::string("Moving_Status: "), mx2ext_msgs_.Moving_Status);
  log(std::string("Present_PWM: "), mx2ext_msgs_.Present_PWM);
  log(std::string("Present_Current: "), mx2ext_msgs_.Present_Current);
  log(std::string("Present_Velocity: "), mx2ext_msgs_.Present_Velocity);
  log(std::string("Present_Position: "), mx2ext_msgs_.Present_Position);
  log(std::string("Velocity_Trajectory: "), mx2ext_msgs_.Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), mx2ext_msgs_.Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), mx2ext_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), mx2ext_msgs_.Present_Temperature);
}

void QNode::EX()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), ex_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), ex_msgs_.Firmware_Version);
  log(std::string("ID: "), ex_msgs_.ID);
  log(std::string("Baud_Rate: "), ex_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), ex_msgs_.Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), ex_msgs_.CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), ex_msgs_.CCW_Angle_Limit);
  log(std::string("Drive_Mode: "), ex_msgs_.Drive_Mode);
  log(std::string("Temperature_Limit: "), ex_msgs_.Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), ex_msgs_.Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), ex_msgs_.Max_Voltage_Limit);
  log(std::string("Max_Torque: "), ex_msgs_.Max_Torque);
  log(std::string("Status_Return_Level: "), ex_msgs_.Status_Return_Level);
  log(std::string("Alarm_LED: "), ex_msgs_.Alarm_LED);
  log(std::string("Shutdown: "), ex_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), ex_msgs_.Torque_Enable);
  log(std::string("LED: "), ex_msgs_.LED);
  log(std::string("CW_Compliance_Margin: "), ex_msgs_.CW_Compliance_Margin);
  log(std::string("CCW_Compliance_Margin: "), ex_msgs_.CCW_Compliance_Margin);
  log(std::string("CW_Compliance_Slope: "), ex_msgs_.CW_Compliance_Slope);
  log(std::string("CCW_Compliance_Slope: "), ex_msgs_.CCW_Compliance_Slope);
  log(std::string("Goal_Position: "), ex_msgs_.Goal_Position);
  log(std::string("Moving_Speed: "), ex_msgs_.Moving_Speed);
  log(std::string("Torque_Limit: "), ex_msgs_.Torque_Limit);
  log(std::string("Present_Position: "), ex_msgs_.Present_Position);
  log(std::string("Present_Speed: "), ex_msgs_.Present_Speed);
  log(std::string("Present_Load: "), ex_msgs_.Present_Load);
  log(std::string("Present_Voltage: "), ex_msgs_.Present_Voltage);
  log(std::string("Present_Temperature: "), ex_msgs_.Present_Temperature);
  log(std::string("Registered: "), ex_msgs_.Registered);
  log(std::string("Moving: "), ex_msgs_.Moving);
  log(std::string("Lock: "), ex_msgs_.Lock);
  log(std::string("Punch: "), ex_msgs_.Punch);
  log(std::string("Sensored_Current: "), ex_msgs_.Sensored_Current);
}

void QNode::XL320()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), xl320_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), xl320_msgs_.Firmware_Version);
  log(std::string("ID: "), xl320_msgs_.ID);
  log(std::string("Baud_Rate: "), xl320_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), xl320_msgs_.Return_Delay_Time);
  log(std::string("CW_Angle_Limit: "), xl320_msgs_.CW_Angle_Limit);
  log(std::string("CCW_Angle_Limit: "), xl320_msgs_.CCW_Angle_Limit);
  log(std::string("Control_Mode: "), xl320_msgs_.Control_Mode);
  log(std::string("Temperature_Limit: "), xl320_msgs_.Temperature_Limit);
  log(std::string("Min_Voltage_Limit: "), xl320_msgs_.Min_Voltage_Limit);
  log(std::string("Max_Voltage_Limit: "), xl320_msgs_.Max_Voltage_Limit);
  log(std::string("Max_Torque: "), xl320_msgs_.Max_Torque);
  log(std::string("Status_Return_Level: "), xl320_msgs_.Status_Return_Level);
  log(std::string("Shutdown: "), xl320_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), xl320_msgs_.Torque_Enable);
  log(std::string("LED: "), xl320_msgs_.LED);
  log(std::string("D_gain: "), xl320_msgs_.D_gain);
  log(std::string("I_gain: "), xl320_msgs_.I_gain);
  log(std::string("P_gain: "), xl320_msgs_.P_gain);
  log(std::string("Goal_Position: "), xl320_msgs_.Goal_Position);
  log(std::string("Moving_Speed: "), xl320_msgs_.Moving_Speed);
  log(std::string("Torque_Limit: "), xl320_msgs_.Torque_Limit);
  log(std::string("Present_Position: "), xl320_msgs_.Present_Position);
  log(std::string("Present_Speed: "), xl320_msgs_.Present_Speed);
  log(std::string("Present_Load: "), xl320_msgs_.Present_Load);
  log(std::string("Present_Voltage: "), xl320_msgs_.Present_Voltage);
  log(std::string("Present_Temperature: "), xl320_msgs_.Present_Temperature);
  log(std::string("Registered: "), xl320_msgs_.Registered);
  log(std::string("Moving: "), xl320_msgs_.Moving);
  log(std::string("Hardware_Error_Status: "), xl320_msgs_.Hardware_Error_Status);
  log(std::string("Punch: "), xl320_msgs_.Punch);
}

void QNode::XL()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), xl_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), xl_msgs_.Firmware_Version);
  log(std::string("ID: "), xl_msgs_.ID);
  log(std::string("Baud_Rate: "), xl_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), xl_msgs_.Return_Delay_Time);
  log(std::string("Drive_Mode: "), xl_msgs_.Drive_Mode);
  log(std::string("Operating_Mode: "), xl_msgs_.Operating_Mode);
  log(std::string("Secondary_ID: "), xl_msgs_.Secondary_ID);
  log(std::string("Protocol_Version: "), xl_msgs_.Protocol_Version);
  log(std::string("Homing_Offset: "), xl_msgs_.Homing_Offset);
  log(std::string("Moving_Threshold: "), xl_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), xl_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), xl_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), xl_msgs_.Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), xl_msgs_.PWM_Limit);
  log(std::string("Acceleration_Limit: "), xl_msgs_.Acceleration_Limit);
  log(std::string("Velocity_Limit: "), xl_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), xl_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), xl_msgs_.Min_Position_Limit);
  log(std::string("Shutdown: "), xl_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), xl_msgs_.Torque_Enable);
  log(std::string("LED: "), xl_msgs_.LED);
  log(std::string("Status_Return_Level: "), xl_msgs_.Status_Return_Level);
  log(std::string("Registered_Instruction: "), xl_msgs_.Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), xl_msgs_.Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), xl_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), xl_msgs_.Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), xl_msgs_.Position_D_Gain);
  log(std::string("Position_I_Gain: "), xl_msgs_.Position_I_Gain);
  log(std::string("Position_P_Gain: "), xl_msgs_.Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), xl_msgs_.Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), xl_msgs_.Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), xl_msgs_.Bus_Watchdog);
  log(std::string("Goal_PWM: "), xl_msgs_.Goal_PWM);
  log(std::string("Goal_Velocity: "), xl_msgs_.Goal_Velocity);
  log(std::string("Profile_Acceleration: "), xl_msgs_.Profile_Acceleration);
  log(std::string("Profile_Velocity: "), xl_msgs_.Profile_Velocity);
  log(std::string("Goal_Position: "), xl_msgs_.Goal_Position);
  log(std::string("Realtime_Tick: "), xl_msgs_.Realtime_Tick);
  log(std::string("Moving: "), xl_msgs_.Moving);
  log(std::string("Moving_Status: "), xl_msgs_.Moving_Status);
  log(std::string("Present_PWM: "), xl_msgs_.Present_PWM);
  log(std::string("Present_Load: "), xl_msgs_.Present_Load);
  log(std::string("Present_Velocity: "), xl_msgs_.Present_Velocity);
  log(std::string("Present_Position: "), xl_msgs_.Present_Position);
  log(std::string("Velocity_Trajectory: "), xl_msgs_.Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), xl_msgs_.Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), xl_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), xl_msgs_.Present_Temperature);
}

void QNode::XM()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), xm_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), xm_msgs_.Firmware_Version);
  log(std::string("ID: "), xm_msgs_.ID);
  log(std::string("Baud_Rate: "), xm_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), xm_msgs_.Return_Delay_Time);
  log(std::string("Drive_Mode: "), xm_msgs_.Drive_Mode);
  log(std::string("Operating_Mode: "), xm_msgs_.Operating_Mode);
  log(std::string("Secondary_ID: "), xm_msgs_.Secondary_ID);
  log(std::string("Protocol_Version: "), xm_msgs_.Protocol_Version);
  log(std::string("Homing_Offset: "), xm_msgs_.Homing_Offset);
  log(std::string("Moving_Threshold: "), xm_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), xm_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), xm_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), xm_msgs_.Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), xm_msgs_.PWM_Limit);
  log(std::string("Current_Limit: "), xm_msgs_.Current_Limit);
  log(std::string("Acceleration_Limit: "), xm_msgs_.Acceleration_Limit);
  log(std::string("Velocity_Limit: "), xm_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), xm_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), xm_msgs_.Min_Position_Limit);
  log(std::string("Shutdown: "), xm_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), xm_msgs_.Torque_Enable);
  log(std::string("LED: "), xm_msgs_.LED);
  log(std::string("Status_Return_Level: "), xm_msgs_.Status_Return_Level);
  log(std::string("Registered_Instruction: "), xm_msgs_.Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), xm_msgs_.Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), xm_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), xm_msgs_.Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), xm_msgs_.Position_D_Gain);
  log(std::string("Position_I_Gain: "), xm_msgs_.Position_I_Gain);
  log(std::string("Position_P_Gain: "), xm_msgs_.Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), xm_msgs_.Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), xm_msgs_.Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), xm_msgs_.Bus_Watchdog);
  log(std::string("Goal_PWM: "), xm_msgs_.Goal_PWM);
  log(std::string("Goal_Current: "), xm_msgs_.Goal_Current);
  log(std::string("Goal_Velocity: "), xm_msgs_.Goal_Velocity);
  log(std::string("Profile_Acceleration: "), xm_msgs_.Profile_Acceleration);
  log(std::string("Profile_Velocity: "), xm_msgs_.Profile_Velocity);
  log(std::string("Goal_Position: "), xm_msgs_.Goal_Position);
  log(std::string("Realtime_Tick: "), xm_msgs_.Realtime_Tick);
  log(std::string("Moving: "), xm_msgs_.Moving);
  log(std::string("Moving_Status: "), xm_msgs_.Moving_Status);
  log(std::string("Present_PWM: "), xm_msgs_.Present_PWM);
  log(std::string("Present_Current: "), xm_msgs_.Present_Current);
  log(std::string("Present_Velocity: "), xm_msgs_.Present_Velocity);
  log(std::string("Present_Position: "), xm_msgs_.Present_Position);
  log(std::string("Velocity_Trajectory: "), xm_msgs_.Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), xm_msgs_.Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), xm_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), xm_msgs_.Present_Temperature);
}

void QNode::XMExt()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), xmext_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), xmext_msgs_.Firmware_Version);
  log(std::string("ID: "), xmext_msgs_.ID);
  log(std::string("Baud_Rate: "), xmext_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), xmext_msgs_.Return_Delay_Time);
  log(std::string("Drive_Mode: "), xmext_msgs_.Drive_Mode);
  log(std::string("Operating_Mode: "), xmext_msgs_.Operating_Mode);
  log(std::string("Secondary_ID: "), xmext_msgs_.Secondary_ID);
  log(std::string("Protocol_Version: "), xmext_msgs_.Protocol_Version);
  log(std::string("Homing_Offset: "), xmext_msgs_.Homing_Offset);
  log(std::string("Moving_Threshold: "), xmext_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), xmext_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), xmext_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), xmext_msgs_.Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), xmext_msgs_.PWM_Limit);
  log(std::string("Current_Limit: "), xmext_msgs_.Current_Limit);
  log(std::string("Acceleration_Limit: "), xmext_msgs_.Acceleration_Limit);
  log(std::string("Velocity_Limit: "), xmext_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), xmext_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), xmext_msgs_.Min_Position_Limit);
  log(std::string("External_Port_Mode_1: "), xmext_msgs_.External_Port_Mode_1);
  log(std::string("External_Port_Mode_2: "), xmext_msgs_.External_Port_Mode_2);
  log(std::string("External_Port_Mode_3: "), xmext_msgs_.External_Port_Mode_3);
  log(std::string("Shutdown: "), xmext_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), xmext_msgs_.Torque_Enable);
  log(std::string("LED: "), xmext_msgs_.LED);
  log(std::string("Status_Return_Level: "), xmext_msgs_.Status_Return_Level);
  log(std::string("Registered_Instruction: "), xmext_msgs_.Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), xmext_msgs_.Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), xmext_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), xmext_msgs_.Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), xmext_msgs_.Position_D_Gain);
  log(std::string("Position_I_Gain: "), xmext_msgs_.Position_I_Gain);
  log(std::string("Position_P_Gain: "), xmext_msgs_.Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), xmext_msgs_.Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), xmext_msgs_.Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), xmext_msgs_.Bus_Watchdog);
  log(std::string("Goal_PWM: "), xmext_msgs_.Goal_PWM);
  log(std::string("Goal_Current: "), xmext_msgs_.Goal_Current);
  log(std::string("Goal_Velocity: "), xmext_msgs_.Goal_Velocity);
  log(std::string("Profile_Acceleration: "), xmext_msgs_.Profile_Acceleration);
  log(std::string("Profile_Velocity: "), xmext_msgs_.Profile_Velocity);
  log(std::string("Goal_Position: "), xmext_msgs_.Goal_Position);
  log(std::string("Realtime_Tick: "), xmext_msgs_.Realtime_Tick);
  log(std::string("Moving: "), xmext_msgs_.Moving);
  log(std::string("Moving_Status: "), xmext_msgs_.Moving_Status);
  log(std::string("Present_PWM: "), xmext_msgs_.Present_PWM);
  log(std::string("Present_Current: "), xmext_msgs_.Present_Current);
  log(std::string("Present_Velocity: "), xmext_msgs_.Present_Velocity);
  log(std::string("Present_Position: "), xmext_msgs_.Present_Position);
  log(std::string("Velocity_Trajectory: "), xmext_msgs_.Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), xmext_msgs_.Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), xmext_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), xmext_msgs_.Present_Temperature);
}

void QNode::XH()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), xh_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), xh_msgs_.Firmware_Version);
  log(std::string("ID: "), xh_msgs_.ID);
  log(std::string("Baud_Rate: "), xh_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), xh_msgs_.Return_Delay_Time);
  log(std::string("Drive_Mode: "), xh_msgs_.Drive_Mode);
  log(std::string("Operating_Mode: "), xh_msgs_.Operating_Mode);
  log(std::string("Secondary_ID: "), xh_msgs_.Secondary_ID);
  log(std::string("Protocol_Version: "), xh_msgs_.Protocol_Version);
  log(std::string("Homing_Offset: "), xh_msgs_.Homing_Offset);
  log(std::string("Moving_Threshold: "), xh_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), xh_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), xh_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), xh_msgs_.Min_Voltage_Limit);
  log(std::string("PWM_Limit: "), xh_msgs_.PWM_Limit);
  log(std::string("Current_Limit: "), xh_msgs_.Current_Limit);
  log(std::string("Acceleration_Limit: "), xh_msgs_.Acceleration_Limit);
  log(std::string("Velocity_Limit: "), xh_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), xh_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), xh_msgs_.Min_Position_Limit);
  log(std::string("Shutdown: "), xh_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), xh_msgs_.Torque_Enable);
  log(std::string("LED: "), xh_msgs_.LED);
  log(std::string("Status_Return_Level: "), xh_msgs_.Status_Return_Level);
  log(std::string("Registered_Instruction: "), xh_msgs_.Registered_Instruction);
  log(std::string("Hardware_Error_Status: "), xh_msgs_.Hardware_Error_Status);
  log(std::string("Velocity_I_Gain: "), xh_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), xh_msgs_.Velocity_P_Gain);
  log(std::string("Position_D_Gain: "), xh_msgs_.Position_D_Gain);
  log(std::string("Position_I_Gain: "), xh_msgs_.Position_I_Gain);
  log(std::string("Position_P_Gain: "), xh_msgs_.Position_P_Gain);
  log(std::string("Feedforward_2nd_Gain: "), xh_msgs_.Feedforward_2nd_Gain);
  log(std::string("Feedforward_1st_Gain: "), xh_msgs_.Feedforward_1st_Gain);
  log(std::string("Bus_Watchdog: "), xh_msgs_.Bus_Watchdog);
  log(std::string("Goal_PWM: "), xh_msgs_.Goal_PWM);
  log(std::string("Goal_Current: "), xh_msgs_.Goal_Current);
  log(std::string("Goal_Velocity: "), xh_msgs_.Goal_Velocity);
  log(std::string("Profile_Acceleration: "), xh_msgs_.Profile_Acceleration);
  log(std::string("Profile_Velocity: "), xh_msgs_.Profile_Velocity);
  log(std::string("Goal_Position: "), xh_msgs_.Goal_Position);
  log(std::string("Realtime_Tick: "), xh_msgs_.Realtime_Tick);
  log(std::string("Moving: "), xh_msgs_.Moving);
  log(std::string("Moving_Status: "), xh_msgs_.Moving_Status);
  log(std::string("Present_PWM: "), xh_msgs_.Present_PWM);
  log(std::string("Present_Current: "), xh_msgs_.Present_Current);
  log(std::string("Present_Velocity: "), xh_msgs_.Present_Velocity);
  log(std::string("Present_Position: "), xh_msgs_.Present_Position);
  log(std::string("Velocity_Trajectory: "), xh_msgs_.Velocity_Trajectory);
  log(std::string("Position_Trajectory: "), xh_msgs_.Position_Trajectory);
  log(std::string("Present_Input_Voltage: "), xh_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), xh_msgs_.Present_Temperature);
}

void QNode::PRO()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), pro_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), pro_msgs_.Firmware_Version);
  log(std::string("ID: "), pro_msgs_.ID);
  log(std::string("Baud_Rate: "), pro_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), pro_msgs_.Return_Delay_Time);
  log(std::string("Operating_Mode: "), pro_msgs_.Operating_Mode);
  log(std::string("Moving_Threshold: "), pro_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), pro_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), pro_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), pro_msgs_.Min_Voltage_Limit);
  log(std::string("Acceleration_Limit: "), pro_msgs_.Acceleration_Limit);
  log(std::string("Torque_Limit: "), pro_msgs_.Torque_Limit);
  log(std::string("Velocity_Limit: "), pro_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), pro_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), pro_msgs_.Min_Position_Limit);
  log(std::string("External_Port_Mode_1: "), pro_msgs_.External_Port_Mode_1);
  log(std::string("External_Port_Mode_2: "), pro_msgs_.External_Port_Mode_2);
  log(std::string("External_Port_Mode_3: "), pro_msgs_.External_Port_Mode_3);
  log(std::string("External_Port_Mode_4: "), pro_msgs_.External_Port_Mode_4);
  log(std::string("Shutdown: "), pro_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), pro_msgs_.Torque_Enable);
  log(std::string("LED_RED: "), pro_msgs_.LED_RED);
  log(std::string("LED_GREEN: "), pro_msgs_.LED_GREEN);
  log(std::string("LED_BLUE: "), pro_msgs_.LED_BLUE);
  log(std::string("Velocity_I_Gain: "), pro_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), pro_msgs_.Velocity_P_Gain);
  log(std::string("Position_P_Gain: "), pro_msgs_.Position_P_Gain);
  log(std::string("Goal_Position: "), pro_msgs_.Goal_Position);
  log(std::string("Goal_Velocity: "), pro_msgs_.Goal_Velocity);
  log(std::string("Goal_Torque: "), pro_msgs_.Goal_Torque);
  log(std::string("Moving: "), pro_msgs_.Moving);
  log(std::string("Present_Position: "), pro_msgs_.Present_Position);
  log(std::string("Present_Velocity: "), pro_msgs_.Present_Velocity);
  log(std::string("Present_Current: "), pro_msgs_.Present_Current);
  log(std::string("Present_Input_Voltage: "), pro_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), pro_msgs_.Present_Temperature);
  log(std::string("Registered_Instruction: "), pro_msgs_.Registered_Instruction);
  log(std::string("Status_Return_Level: "), pro_msgs_.Status_Return_Level);
  log(std::string("Hardware_Error_Status: "), pro_msgs_.Hardware_Error_Status);
}

void QNode::PROExt()
{
  log(std::string("< EEPROM >"));
  log(std::string("Model_Number: "), proext_msgs_.Model_Number);
  log(std::string("Firmware_Version: "), proext_msgs_.Firmware_Version);
  log(std::string("ID: "), proext_msgs_.ID);
  log(std::string("Baud_Rate: "), proext_msgs_.Baud_Rate);
  log(std::string("Return_Delay_Time: "), proext_msgs_.Return_Delay_Time);
  log(std::string("Operating_Mode: "), proext_msgs_.Operating_Mode);
  log(std::string("Homing_Offset: "), proext_msgs_.Homing_Offset);
  log(std::string("Moving_Threshold: "), proext_msgs_.Moving_Threshold);
  log(std::string("Temperature_Limit: "), proext_msgs_.Temperature_Limit);
  log(std::string("Max_Voltage_Limit: "), proext_msgs_.Max_Voltage_Limit);
  log(std::string("Min_Voltage_Limit: "), proext_msgs_.Min_Voltage_Limit);
  log(std::string("Acceleration_Limit: "), proext_msgs_.Acceleration_Limit);
  log(std::string("Torque_Limit: "), proext_msgs_.Torque_Limit);
  log(std::string("Velocity_Limit: "), proext_msgs_.Velocity_Limit);
  log(std::string("Max_Position_Limit: "), proext_msgs_.Max_Position_Limit);
  log(std::string("Min_Position_Limit: "), proext_msgs_.Min_Position_Limit);
  log(std::string("External_Port_Mode_1: "), proext_msgs_.External_Port_Mode_1);
  log(std::string("External_Port_Mode_2: "), proext_msgs_.External_Port_Mode_2);
  log(std::string("External_Port_Mode_3: "), proext_msgs_.External_Port_Mode_3);
  log(std::string("External_Port_Mode_4: "), proext_msgs_.External_Port_Mode_4);
  log(std::string("Shutdown: "), proext_msgs_.Shutdown);
  log(std::string(""));
  log(std::string("< RAM >"));
  log(std::string("Torque_Enable: "), proext_msgs_.Torque_Enable);
  log(std::string("LED_RED: "), proext_msgs_.LED_RED);
  log(std::string("LED_GREEN: "), proext_msgs_.LED_GREEN);
  log(std::string("LED_BLUE: "), proext_msgs_.LED_BLUE);
  log(std::string("Velocity_I_Gain: "), proext_msgs_.Velocity_I_Gain);
  log(std::string("Velocity_P_Gain: "), proext_msgs_.Velocity_P_Gain);
  log(std::string("Position_P_Gain: "), proext_msgs_.Position_P_Gain);
  log(std::string("Goal_Position: "), proext_msgs_.Goal_Position);
  log(std::string("Goal_Velocity: "), proext_msgs_.Goal_Velocity);
  log(std::string("Goal_Torque: "), proext_msgs_.Goal_Torque);
  log(std::string("Moving: "), proext_msgs_.Moving);
  log(std::string("Present_Position: "), proext_msgs_.Present_Position);
  log(std::string("Present_Velocity: "), proext_msgs_.Present_Velocity);
  log(std::string("Present_Current: "), proext_msgs_.Present_Current);
  log(std::string("Present_Input_Voltage: "), proext_msgs_.Present_Input_Voltage);
  log(std::string("Present_Temperature: "), proext_msgs_.Present_Temperature);
  log(std::string("Registered_Instruction: "), proext_msgs_.Registered_Instruction);
  log(std::string("Status_Return_Level: "), proext_msgs_.Status_Return_Level);
  log(std::string("Hardware_Error_Status: "), proext_msgs_.Hardware_Error_Status);
}

void QNode::AXStatusMsgCallback(const dynamixel_workbench_msgs::AX::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  ax_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::RXStatusMsgCallback(const dynamixel_workbench_msgs::RX::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  rx_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::MXStatusMsgCallback(const dynamixel_workbench_msgs::MX::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  mx_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::MXExtStatusMsgCallback(const dynamixel_workbench_msgs::MXExt::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  mxext_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::MX2StatusMsgCallback(const dynamixel_workbench_msgs::MX2::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  mx2_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::MX2ExtStatusMsgCallback(const dynamixel_workbench_msgs::MX2Ext::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  mx2ext_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::EXStatusMsgCallback(const dynamixel_workbench_msgs::EX::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  ex_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::XL320StatusMsgCallback(const dynamixel_workbench_msgs::XL320::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  xl320_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::XLStatusMsgCallback(const dynamixel_workbench_msgs::XL::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  xl_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::XMStatusMsgCallback(const dynamixel_workbench_msgs::XM::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  xm_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::XMExtStatusMsgCallback(const dynamixel_workbench_msgs::XMExt::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  xmext_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::XHStatusMsgCallback(const dynamixel_workbench_msgs::XH::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  xh_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::PROStatusMsgCallback(const dynamixel_workbench_msgs::PRO::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  pro_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}

void QNode::PROExtStatusMsgCallback(const dynamixel_workbench_msgs::PROExt::ConstPtr &msg)
{
  pthread_mutex_lock(&mutex);
  proext_msgs_ = *msg;
  pthread_mutex_unlock(&mutex);
}
