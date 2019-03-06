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

#include "dynamixel_workbench_single_manager/single_dynamixel_monitor.h"

using namespace single_dynamixel_monitor;

inline void millis(uint16_t msec)
{
  usleep(1000*msec);
}

SingleDynamixelMonitor::SingleDynamixelMonitor(void)
  :dxl_baud_rate_(0),
   dxl_id_(0)
{
  // Check Dynamixel Ping or Scan (default : Scan (1~253))

  // Dynamixel Monitor variable
  bool use_ping  = node_handle_.param<bool>("ping", false);
  int ping_id    = node_handle_.param<int>("ping_id", 1);
  int scan_range = node_handle_.param<int>("scan_range", 200);

  device_name_   = node_handle_.param<std::string>("device_name", "/dev/ttyUSB0");
  dxl_baud_rate_ = node_handle_.param<int>("baud_rate", 57600);

  // Init Dynamixel Driver
  dynamixel_driver_ = new DynamixelDriver;

  dynamixel_driver_->init(device_name_.c_str(), dxl_baud_rate_);


  if(use_ping == true)
  {
    uint16_t model_number = 0;
    if (dynamixel_driver_->ping(ping_id, &model_number))
    {
      dxl_id_ = ping_id;
      printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d [VERSION] %.1f\n",
               dxl_id_, dynamixel_driver_->getModelName(dxl_id_), dxl_baud_rate_, dynamixel_driver_->getProtocolVersion());
    }
    else
    {
      printf("Please Check USB Port authorization and\n");
      printf("Baudrate [ex : 9600, 57600, 115200, 1000000, 2000000]\n");
      printf("...Failed to find dynamixel!\n");

      ros::shutdown();
      exit(1);
    }
  }
  else
  {
    uint8_t id_cnt = 0;
    if (dynamixel_driver_->scan(&dxl_id_, &id_cnt, scan_range))
    {
      printf("[ID] %u, [Model Name] %s, [BAUD RATE] %d [VERSION] %.1f\n",
               dxl_id_, dynamixel_driver_->getModelName(dxl_id_), dxl_baud_rate_, dynamixel_driver_->getProtocolVersion());
    }
    else
    {
      printf("Please Check USB Port authorization and\n");
      printf("Baudrate [ex : 9600, 57600, 115200, 1000000, 2000000]\n");
      printf("...Failed to find dynamixel!\n");

      ros::shutdown();
      exit(1);
    }
  }

  initDynamixelStatePublisher();
  initDynamixelInfoServer();
  initDynamixelCommandServer();

  printf("dynamixel_workbench_single_manager : Init Success!\n");
}

SingleDynamixelMonitor::~SingleDynamixelMonitor(void)
{

}

void SingleDynamixelMonitor::initSingleDynamixelMonitor(void)
{

}

void SingleDynamixelMonitor::shutdownSingleDynamixelMonitor(void)
{
  dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

  ros::shutdown();
}

void SingleDynamixelMonitor::initDynamixelStatePublisher()
{
  char* model_name = dynamixel_driver_->getModelName(dxl_id_);

  if (!strncmp(model_name, "AX", strlen("AX")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::AX>("dynamixel/" + std::string("AX"), 10);
  }
  else if (!strncmp(model_name, "RX", strlen("RX")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::RX>("dynamixel/" + std::string("RX"), 10);
  }
  else if (!strncmp(model_name, "MX", strlen("MX")))
  {
    if (!strncmp(model_name, "MX-12W", strlen(model_name)) ||
        !strncmp(model_name, "MX-28" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MX>("dynamixel/" + std::string("MX"), 10);
    }
    else if (!strncmp(model_name, "MX-64", strlen(model_name)) ||
             !strncmp(model_name, "MX-106" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MXExt>("dynamixel/" + std::string("MX"), 10);
    }
    else if (!strncmp(model_name, "MX-28-2", strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MX2>("dynamixel/" + std::string("MX"), 10);
    }
    else if (!strncmp(model_name, "MX-64-2", strlen(model_name)) ||
             !strncmp(model_name, "MX-106-2" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::MX2Ext>("dynamixel/" + std::string("MX"), 10);
    }
  }
  else if (!strncmp(model_name, "EX", strlen("EX")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::EX>("dynamixel/" + std::string("EX"), 10);
  }
  else if (!strncmp(model_name, "XL", strlen("XL")))
  {
    if (!strncmp(model_name, "XL-320", strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XL320>("dynamixel/" + std::string("XL"), 10);
    }
    else if (!strncmp(model_name, "XL430-W250", strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XL>("dynamixel/" + std::string("XL"), 10);
    }
  }
  else if (!strncmp(model_name, "XM", strlen("XM")))
  {
    if (!strncmp(model_name, "XM430-W210", strlen(model_name)) ||
        !strncmp(model_name, "XM430-W350" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XM>("dynamixel/" + std::string("XM"), 10);
    }
    else if (!strncmp(model_name, "XM540-W150", strlen(model_name)) ||
             !strncmp(model_name, "XM540-W270" , strlen(model_name)))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XMExt>("dynamixel/" + std::string("XM"), 10);
    }
  }
  else if (!strncmp(model_name, "XH", strlen("XH")))
  {
    dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::XH>("dynamixel/" + std::string("XH"), 10);
  }
  else if (!strncmp(model_name, "PRO", strlen("PRO")))
  {
    if ((!strncmp(model_name, "PRO_L42_10_S300_R", strlen(model_name))))
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::PRO>("dynamixel/" + std::string("PRO"), 10);
    }
    else
    {
      dynamixel_status_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::PROExt>("dynamixel/" + std::string("PRO"), 10);
    }
  }
}

void SingleDynamixelMonitor::initDynamixelInfoServer(void)
{
  dynamixel_info_server_ = node_handle_.advertiseService("dynamixel/info", &SingleDynamixelMonitor::dynamixelInfoMsgCallback, this);
}

void SingleDynamixelMonitor::initDynamixelCommandServer(void)
{
  dynamixel_command_server_ = node_handle_.advertiseService("dynamixel/command", &SingleDynamixelMonitor::dynamixelCommandMsgCallback, this);
}

bool SingleDynamixelMonitor::showDynamixelControlTable(void)
{
  bool comm_result = false;
  int32_t torque_status = 0;
  uint16_t torque_enable_address = 0;
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);

  for (int item_num = 0; item_num < dynamixel_driver_->getTheNumberOfItem(dxl_id_); item_num++)
  {
    if (!strncmp(item_ptr[item_num].item_name, "Torque_Enable", strlen("Torque_Enable")))
    {
      torque_enable_address = item_num;
    }
  }

  comm_result = dynamixel_driver_->readRegister(dxl_id_, "Torque_Enable", &torque_status);

  for (int item_num = 0; item_num < dynamixel_driver_->getTheNumberOfItem(dxl_id_); item_num++)
  {
    if (torque_status == false)
    {
      printf("%s\n", item_ptr[item_num].item_name);
    }
    else
    {
      if (item_num >= torque_enable_address)
        printf("%s\n", item_ptr[item_num].item_name);
    }
  }

  return comm_result;
}

bool SingleDynamixelMonitor::checkValidationCommand(std::string cmd)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);

  for (int item_num = 0; item_num < dynamixel_driver_->getTheNumberOfItem(dxl_id_); item_num++)
  {
    if (!strncmp(item_ptr[item_num].item_name, cmd.c_str(), strlen(item_ptr[item_num].item_name)))
      return true;
  }

  printf("Please Check DYNAMXEL Address Name('table')\n");
  return false;
}

bool SingleDynamixelMonitor::changeId(uint8_t new_id)
{
  bool comm_result = false;

  if (new_id > 0 && new_id < 254)
  {
    comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

    comm_result = dynamixel_driver_->writeRegister(dxl_id_, "ID", new_id);
    millis(1000);

    uint16_t model_number = 0;
    if (comm_result && dynamixel_driver_->ping(new_id, &model_number))
    {
      dxl_id_ = new_id;

      printf("...Succeeded to set Dynamixel ID [%u]\n", dxl_id_);
      return true;
    }
    else
    {
      printf("...Failed to change ID!\n");
      return false;
    }
  }
  else
  {
    printf("Dynamixel ID can be set 1~253\n");
    return comm_result;
  }
}

bool SingleDynamixelMonitor::changeBaudrate(uint32_t new_baud_rate)
{
  bool comm_result = false;
  bool check_baud_rate = false;

  uint64_t baud_rate_list[14] = {9600, 19200, 57600, 115200, 200000, 250000, 400000, 500000,
                                 1000000, 2000000, 3000000, 4000000, 4500000, 10500000};

  for (int i = 0; i < 14; i++)
  {
    if (baud_rate_list[i] == new_baud_rate)
      check_baud_rate = true;
  }

  if (check_baud_rate == false)
  {
    printf("Failed to change [ BAUD RATE: %d ]\n", new_baud_rate);
    printf("Valid baud rate is [9600, 57600, 115200, 1000000, 2000000,...]\n");
    printf("You can choose other baud rate in GUI,\n");

    return false;
  }
  else
  {
    comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

    if (dynamixel_driver_->getProtocolVersion() == 1.0)
    {
      if (new_baud_rate == 9600)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 207);
      else if (new_baud_rate == 19200)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 103);
      else if (new_baud_rate == 57600)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 34);
      else if (new_baud_rate == 115200)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 16);
      else if (new_baud_rate == 200000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 9);
      else if (new_baud_rate == 250000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 7);
      else if (new_baud_rate == 400000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 4);
      else if (new_baud_rate == 500000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 3);
      else if (new_baud_rate == 1000000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 1);
      else
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 34);
    }
    else if (dynamixel_driver_->getProtocolVersion() == 2.0)
    {
      if (new_baud_rate == 9600)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 0);
      else if (new_baud_rate == 57600)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 1);
      else if (new_baud_rate == 115200)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 2);
      else if (new_baud_rate == 1000000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 3);
      else if (new_baud_rate == 2000000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 4);
      else if (new_baud_rate == 3000000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 5);
      else if (new_baud_rate == 4000000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 6);
      else if (new_baud_rate == 4500000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 7);
      else if (new_baud_rate == 10500000)
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 8);
      else
        comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Baud_Rate", 1);
    }

    millis(2000);

    if (comm_result)
    {
      if (dynamixel_driver_->setBaudrate(new_baud_rate))
      {
        printf("Success to change baudrate! [ BAUD RATE: %d ]\n", new_baud_rate);
        return true;
      }
      else
      {
        printf("Failed to change baudrate!\n");
        return false;
      }
    }
    else
    {
      printf("Failed to change baudrate!\n");
      return false;
    }
  }
}

bool SingleDynamixelMonitor::changeProtocolVersion(float ver)
{
  bool error = false;
  bool comm_result = false;

  if (ver == 1.0 || ver == 2.0)
  {
    comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", false);

    comm_result = dynamixel_driver_->writeRegister(dxl_id_, "Protocol_Version", (int)(ver));
    millis(2000);

    if (comm_result)
    {
      if (dynamixel_driver_->setPacketHandler(ver))
      {
        printf("Success to change protocol version [PROTOCOL VERSION: %.1f]\n", dynamixel_driver_->getProtocolVersion());
        return true;
      }
      else
        return false;
    }
    else
    {
      return false;
    }
  }
  else
  {
    printf("Dynamixel supports protocol version [1.0] or [2.0]\n");
    return false;
  }
}

bool SingleDynamixelMonitor::controlLoop(void)
{
  dynamixelStatePublish();

  return true;
}

bool SingleDynamixelMonitor::dynamixelInfoMsgCallback(dynamixel_workbench_msgs::GetDynamixelInfo::Request &req,
                                                   dynamixel_workbench_msgs::GetDynamixelInfo::Response &res)
{
  res.dynamixel_info.load_info.device_name      = device_name_;
  res.dynamixel_info.load_info.baud_rate        = dynamixel_driver_->getBaudrate();
  res.dynamixel_info.load_info.protocol_version = dynamixel_driver_->getProtocolVersion();

  res.dynamixel_info.model_id         = dxl_id_;
  res.dynamixel_info.model_name       = dynamixel_driver_->getModelName(dxl_id_);
  res.dynamixel_info.model_number     = dynamixel_driver_->getModelNum(dxl_id_);

  return true;
}

bool SingleDynamixelMonitor::dynamixelCommandMsgCallback(dynamixel_workbench_msgs::DynamixelCommand::Request &req,
                                                         dynamixel_workbench_msgs::DynamixelCommand::Response &res)
{
  if (req.command == "table")
  {
    if (showDynamixelControlTable())
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "reboot")
  {
    if (dynamixel_driver_->reboot(dxl_id_))
    {
      printf("Succeed to reboot\n");
      res.comm_result = true;
    }
    else
    {
      printf("Failed to reboot\n");
      res.comm_result = false;
    }
  }
  else if (req.command == "factory_reset")
  {
    if (dynamixel_driver_->reset(dxl_id_))
    {
      dxl_id_ = 1;
      printf("Succeed to reset\n");
      res.comm_result = true;
    }
    else
    {
      printf("Failed to reboot\n");
      res.comm_result = false;
    }
  }
  else if (req.command == "torque")
  {
    int32_t value = req.value;

    if (dynamixel_driver_->writeRegister(dxl_id_, "Torque_Enable", value))
      res.comm_result = true;
    else
      res.comm_result = false;
  }
  else if (req.command == "exit")
  {
    shutdownSingleDynamixelMonitor();
  }
  else if (req.command == "addr")
  {
    std::string addr = req.addr_name;
    int64_t value    = req.value;

    if (checkValidationCommand(addr))
    {
      res.comm_result = true;
    }
    else
    {
      res.comm_result = false;
      return false;
    }

    if (addr == "ID")
    {
      if (changeId(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else if (addr == "Baud_Rate")
    {
      if (changeBaudrate(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else if (addr == "Protocol_Version")
    {
      if (changeProtocolVersion(value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
    else
    {
      if (dynamixel_driver_->writeRegister(dxl_id_, addr.c_str(), value))
        res.comm_result = true;
      else
        res.comm_result = false;
    }
  }
  else
  {
    return false;
  }

  return true;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "single_dynamixel_monitor");

  SingleDynamixelMonitor single_dynamixel_monitor;
  ros::Rate loop_rate(1000);

  while (ros::ok())
  {
    single_dynamixel_monitor.controlLoop();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}


void SingleDynamixelMonitor::dynamixelStatePublish(void)
{
  char* model_name = dynamixel_driver_->getModelName(dxl_id_);

  if (!strncmp(model_name, "AX", strlen("AX")))
  {
    AX();
  }
  else if (!strncmp(model_name, "RX", strlen("RX")))
  {
    RX();
  }
  else if (!strncmp(model_name, "MX-12W", strlen(model_name)) ||
           !strncmp(model_name, "MX-28" , strlen(model_name)))
  {
    MX();
  }
  else if (!strncmp(model_name, "MX-64", strlen(model_name)) ||
           !strncmp(model_name, "MX-106" , strlen(model_name)))
  {
    MXExt();
  }
  else if (!strncmp(model_name, "MX-28-2", strlen(model_name)))
  {
    MX2();
  }
  else if (!strncmp(model_name, "MX-64-2", strlen(model_name)) ||
           !strncmp(model_name, "MX-106-2" , strlen(model_name)))
  {
    MX2Ext();
  }
  else if (!strncmp(model_name, "EX", strlen("EX")))
  {
    EX();
  }
  else if (!strncmp(model_name, "XL-320", strlen(model_name)))
  {
    XL320();
  }
  else if (!strncmp(model_name, "XL430-W250", strlen(model_name)))
  {
    XL();
  }
  else if (!strncmp(model_name, "XM430-W210", strlen(model_name)) ||
           !strncmp(model_name, "XM430-W350" , strlen(model_name)))
  {
    XM();
  }
  else if (!strncmp(model_name, "XM540-W150", strlen(model_name)) ||
           !strncmp(model_name, "XM540-W270" , strlen(model_name)))
  {
    XMExt();
  }
  else if (!strncmp(model_name, "XH", strlen("XH")))
  {
    XH();
  }
  else if (!strncmp(model_name, "PRO_L42_10_S300_R", strlen("PRO_L42_10_S300_R")))
  {
    PRO();
  }
  else if (!strncmp(model_name, "PRO", strlen("PRO")))
  {
    PROExt();
  }
}

void SingleDynamixelMonitor::AX(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::AX ax_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      ax_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      ax_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      ax_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      ax_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      ax_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      ax_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      ax_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      ax_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      ax_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      ax_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      ax_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      ax_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      ax_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      ax_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      ax_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      ax_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Margin", strlen("CW_Compliance_Margin")))
      ax_state.CW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Margin", strlen("CCW_Compliance_Margin")))
      ax_state.CCW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Slope", strlen("CW_Compliance_Slope")))
      ax_state.CW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Slope", strlen("CCW_Compliance_Slope")))
      ax_state.CCW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      ax_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      ax_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      ax_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      ax_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      ax_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      ax_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      ax_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      ax_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      ax_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      ax_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      ax_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      ax_state.Punch = read_value;
  }

  dynamixel_status_pub_.publish(ax_state);
}

void SingleDynamixelMonitor::RX(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::RX rx_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      rx_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      rx_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      rx_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      rx_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      rx_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      rx_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      rx_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      rx_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      rx_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      rx_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      rx_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      rx_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      rx_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      rx_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      rx_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      rx_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Margin", strlen("CW_Compliance_Margin")))
      rx_state.CW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Margin", strlen("CCW_Compliance_Margin")))
      rx_state.CCW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Slope", strlen("CW_Compliance_Slope")))
      rx_state.CW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Slope", strlen("CCW_Compliance_Slope")))
      rx_state.CCW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      rx_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      rx_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      rx_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      rx_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      rx_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      rx_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      rx_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      rx_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      rx_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      rx_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      rx_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      rx_state.Punch = read_value;
  }

  dynamixel_status_pub_.publish(rx_state);
}

void SingleDynamixelMonitor::MX(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MX mx_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mx_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mx_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mx_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mx_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mx_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      mx_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      mx_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mx_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mx_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mx_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      mx_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mx_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      mx_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mx_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Multi_Turn_Offset", strlen("Multi_Turn_Offset")))
      mx_state.Multi_Turn_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Resolution_Divider", strlen("Resolution_Divider")))
      mx_state.Resolution_Divider = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mx_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mx_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "D_gain", strlen("D_gain")))
      mx_state.D_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "I_gain", strlen("I_gain")))
      mx_state.I_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "P_gain", strlen("P_gain")))
      mx_state.P_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mx_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      mx_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      mx_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mx_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      mx_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      mx_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      mx_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mx_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      mx_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mx_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      mx_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      mx_state.Punch = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Acceleration", strlen("Goal_Acceleration")))
      mx_state.Goal_Acceleration = read_value;
  }

  dynamixel_status_pub_.publish(mx_state);
}

void SingleDynamixelMonitor::MXExt(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MXExt mxext_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mxext_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mxext_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mxext_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mxext_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mxext_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      mxext_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      mxext_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mxext_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mxext_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mxext_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      mxext_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mxext_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      mxext_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mxext_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Multi_Turn_Offset", strlen("Multi_Turn_Offset")))
      mxext_state.Multi_Turn_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Resolution_Divider", strlen("Resolution_Divider")))
      mxext_state.Resolution_Divider = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mxext_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mxext_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "D_gain", strlen("D_gain")))
      mxext_state.D_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "I_gain", strlen("I_gain")))
      mxext_state.I_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "P_gain", strlen("P_gain")))
      mxext_state.P_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mxext_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      mxext_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      mxext_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mxext_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      mxext_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      mxext_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      mxext_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mxext_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      mxext_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mxext_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      mxext_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      mxext_state.Punch = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Current", strlen("Current")))
      mxext_state.Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Control_Mode_Enable", strlen("Torque_Control_Mode_Enable")))
      mxext_state.Torque_Control_Mode_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Torque", strlen("Goal_Torque")))
      mxext_state.Goal_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Acceleration", strlen("Goal_Acceleration")))
      mxext_state.Goal_Acceleration = read_value;
  }

  dynamixel_status_pub_.publish(mxext_state);
}

void SingleDynamixelMonitor::MX2(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MX2 mx2_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mx2_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mx2_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mx2_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mx2_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mx2_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      mx2_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      mx2_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      mx2_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      mx2_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      mx2_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      mx2_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mx2_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mx2_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mx2_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      mx2_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      mx2_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      mx2_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      mx2_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      mx2_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mx2_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mx2_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mx2_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mx2_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      mx2_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      mx2_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      mx2_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      mx2_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      mx2_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      mx2_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      mx2_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      mx2_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      mx2_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      mx2_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      mx2_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      mx2_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      mx2_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      mx2_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mx2_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      mx2_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mx2_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      mx2_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      mx2_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      mx2_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      mx2_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mx2_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      mx2_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      mx2_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      mx2_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mx2_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(mx2_state);
}

void SingleDynamixelMonitor::MX2Ext(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::MX2Ext mx2ext_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      mx2ext_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      mx2ext_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      mx2ext_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      mx2ext_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      mx2ext_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      mx2ext_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      mx2ext_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      mx2ext_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      mx2ext_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      mx2ext_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      mx2ext_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      mx2ext_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      mx2ext_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      mx2ext_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      mx2ext_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Current_Limit", strlen("Current_Limit")))
      mx2ext_state.Current_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      mx2ext_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      mx2ext_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      mx2ext_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      mx2ext_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      mx2ext_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      mx2ext_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      mx2ext_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      mx2ext_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      mx2ext_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      mx2ext_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      mx2ext_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      mx2ext_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      mx2ext_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      mx2ext_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      mx2ext_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      mx2ext_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      mx2ext_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      mx2ext_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      mx2ext_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Current", strlen("Goal_Current")))
      mx2ext_state.Goal_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      mx2ext_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      mx2ext_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      mx2ext_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      mx2ext_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      mx2ext_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      mx2ext_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      mx2ext_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      mx2ext_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Current", strlen("Present_Current")))
      mx2ext_state.Present_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      mx2ext_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      mx2ext_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      mx2ext_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      mx2ext_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      mx2ext_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      mx2ext_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(mx2ext_state);
}

void SingleDynamixelMonitor::EX(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::EX ex_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      ex_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      ex_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      ex_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      ex_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      ex_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      ex_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      ex_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      ex_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      ex_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      ex_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      ex_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      ex_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      ex_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Alarm_LED", strlen("Alarm_LED")))
      ex_state.Alarm_LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      ex_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      ex_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      ex_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Margin", strlen("CW_Compliance_Margin")))
      ex_state.CW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Margin", strlen("CCW_Compliance_Margin")))
      ex_state.CCW_Compliance_Margin = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Compliance_Slope", strlen("CW_Compliance_Slope")))
      ex_state.CW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Compliance_Slope", strlen("CCW_Compliance_Slope")))
      ex_state.CCW_Compliance_Slope = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      ex_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      ex_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      ex_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      ex_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      ex_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      ex_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      ex_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      ex_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      ex_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      ex_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Lock", strlen("Lock")))
      ex_state.Lock = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      ex_state.Punch = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Sensored_Current", strlen("Sensored_Current")))
      ex_state.Sensored_Current = read_value;
  }

  dynamixel_status_pub_.publish(ex_state);
}

void SingleDynamixelMonitor::XL320(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::XL320 xl320_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      xl320_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      xl320_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      xl320_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      xl320_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      xl320_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CW_Angle_Limit", strlen("CW_Angle_Limit")))
      xl320_state.CW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "CCW_Angle_Limit", strlen("CCW_Angle_Limit")))
      xl320_state.CCW_Angle_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      xl320_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      xl320_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      xl320_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Torque", strlen("Max_Torque")))
      xl320_state.Max_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      xl320_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      xl320_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      xl320_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      xl320_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "D_gain", strlen("D_gain")))
      xl320_state.D_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "I_gain", strlen("I_gain")))
      xl320_state.I_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "P_gain", strlen("P_gain")))
      xl320_state.P_gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      xl320_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Speed", strlen("Moving_Speed")))
      xl320_state.Moving_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      xl320_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      xl320_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Speed", strlen("Present_Speed")))
      xl320_state.Present_Speed = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      xl320_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Voltage", strlen("Present_Voltage")))
      xl320_state.Present_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      xl320_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered", strlen("Registered")))
      xl320_state.Registered = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      xl320_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      xl320_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Punch", strlen("Punch")))
      xl320_state.Punch = read_value;
  }

  dynamixel_status_pub_.publish(xl320_state);
}

void SingleDynamixelMonitor::XL(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::XL xl_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      xl_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      xl_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      xl_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      xl_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      xl_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      xl_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      xl_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      xl_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      xl_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      xl_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      xl_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      xl_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      xl_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      xl_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      xl_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      xl_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      xl_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      xl_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      xl_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      xl_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      xl_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      xl_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      xl_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      xl_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      xl_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      xl_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      xl_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      xl_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      xl_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      xl_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      xl_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      xl_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      xl_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      xl_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      xl_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      xl_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      xl_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      xl_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      xl_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      xl_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      xl_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      xl_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Load", strlen("Present_Load")))
      xl_state.Present_Load = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      xl_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      xl_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      xl_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      xl_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      xl_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      xl_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(xl_state);
}

void SingleDynamixelMonitor::XM(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::XM xm_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      xm_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      xm_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      xm_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      xm_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      xm_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      xm_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      xm_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      xm_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      xm_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      xm_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      xm_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      xm_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      xm_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      xm_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      xm_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Current_Limit", strlen("Current_Limit")))
      xm_state.Current_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      xm_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      xm_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      xm_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      xm_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      xm_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      xm_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      xm_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      xm_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      xm_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      xm_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      xm_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      xm_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      xm_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      xm_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      xm_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      xm_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      xm_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      xm_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      xm_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Current", strlen("Goal_Current")))
      xm_state.Goal_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      xm_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      xm_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      xm_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      xm_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      xm_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      xm_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      xm_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      xm_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Current", strlen("Present_Current")))
      xm_state.Present_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      xm_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      xm_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      xm_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      xm_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      xm_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      xm_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(xm_state);
}

void SingleDynamixelMonitor::XMExt(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::XMExt xmext_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      xmext_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      xmext_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      xmext_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      xmext_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      xmext_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      xmext_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      xmext_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      xmext_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      xmext_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      xmext_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      xmext_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      xmext_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      xmext_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      xmext_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      xmext_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Current_Limit", strlen("Current_Limit")))
      xmext_state.Current_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      xmext_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      xmext_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      xmext_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      xmext_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_1", strlen("External_Port_Mode_1")))
      xmext_state.External_Port_Mode_1 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_2", strlen("External_Port_Mode_2")))
      xmext_state.External_Port_Mode_2 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_3", strlen("External_Port_Mode_3")))
      xmext_state.External_Port_Mode_3 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      xmext_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      xmext_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      xmext_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      xmext_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      xmext_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      xmext_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      xmext_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      xmext_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      xmext_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      xmext_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      xmext_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      xmext_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      xmext_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      xmext_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      xmext_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Current", strlen("Goal_Current")))
      xmext_state.Goal_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      xmext_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      xmext_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      xmext_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      xmext_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      xmext_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      xmext_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      xmext_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      xmext_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Current", strlen("Present_Current")))
      xmext_state.Present_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      xmext_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      xmext_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      xmext_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      xmext_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      xmext_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      xmext_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(xmext_state);
}

void SingleDynamixelMonitor::XH(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::XH xh_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      xh_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      xh_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      xh_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      xh_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      xh_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Drive_Mode", strlen("Drive_Mode")))
      xh_state.Drive_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      xh_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Secondary_ID", strlen("Secondary_ID")))
      xh_state.Secondary_ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Protocol_Version", strlen("Protocol_Version")))
      xh_state.Protocol_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      xh_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      xh_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      xh_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      xh_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      xh_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "PWM_Limit", strlen("PWM_Limit")))
      xh_state.PWM_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Current_Limit", strlen("Current_Limit")))
      xh_state.Current_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      xh_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      xh_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      xh_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      xh_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      xh_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      xh_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED", strlen("LED")))
      xh_state.LED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      xh_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      xh_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      xh_state.Hardware_Error_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      xh_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      xh_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_D_Gain", strlen("Position_D_Gain")))
      xh_state.Position_D_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_I_Gain", strlen("Position_I_Gain")))
      xh_state.Position_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      xh_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_2nd_Gain", strlen("Feedforward_2nd_Gain")))
      xh_state.Feedforward_2nd_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Feedforward_1st_Gain", strlen("Feedforward_1st_Gain")))
      xh_state.Feedforward_1st_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Bus_Watchdog", strlen("Bus_Watchdog")))
      xh_state.Bus_Watchdog = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_PWM", strlen("Goal_PWM")))
      xh_state.Goal_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Current", strlen("Goal_Current")))
      xh_state.Goal_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      xh_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Acceleration", strlen("Profile_Acceleration")))
      xh_state.Profile_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Profile_Velocity", strlen("Profile_Velocity")))
      xh_state.Profile_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      xh_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Realtime_Tick", strlen("Realtime_Tick")))
      xh_state.Realtime_Tick = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      xh_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Status", strlen("Moving_Status")))
      xh_state.Moving_Status = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_PWM", strlen("Present_PWM")))
      xh_state.Present_PWM = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Current", strlen("Present_Current")))
      xh_state.Present_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      xh_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      xh_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Trajectory", strlen("Velocity_Trajectory")))
      xh_state.Velocity_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_Trajectory", strlen("Position_Trajectory")))
      xh_state.Position_Trajectory = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      xh_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      xh_state.Present_Temperature = read_value;
  }

  dynamixel_status_pub_.publish(xh_state);
}

void SingleDynamixelMonitor::PRO(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::PRO pro_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      pro_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      pro_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      pro_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      pro_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      pro_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      pro_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      pro_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      pro_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      pro_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      pro_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      pro_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      pro_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      pro_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      pro_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      pro_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_1", strlen("External_Port_Mode_1")))
      pro_state.External_Port_Mode_1 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_2", strlen("External_Port_Mode_2")))
      pro_state.External_Port_Mode_2 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_3", strlen("External_Port_Mode_3")))
      pro_state.External_Port_Mode_3 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_4", strlen("External_Port_Mode_4")))
      pro_state.External_Port_Mode_4 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      pro_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      pro_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED_RED", strlen("LED_RED")))
      pro_state.LED_RED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED_GREEN", strlen("LED_GREEN")))
      pro_state.LED_GREEN = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED_BLUE", strlen("LED_BLUE")))
      pro_state.LED_BLUE = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      pro_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      pro_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      pro_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      pro_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      pro_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Torque", strlen("Goal_Torque")))
      pro_state.Goal_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Acceleration", strlen("Goal_Acceleration")))
      pro_state.Goal_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      pro_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      pro_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      pro_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Current", strlen("Present_Current")))
      pro_state.Present_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      pro_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      pro_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      pro_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      pro_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      pro_state.Hardware_Error_Status = read_value;
  }

  dynamixel_status_pub_.publish(pro_state);
}

void SingleDynamixelMonitor::PROExt(void)
{
  ControlTableItem* item_ptr = dynamixel_driver_->getControlItemPtr(dxl_id_);
  dynamixel_workbench_msgs::PROExt proext_state;

  uint16_t last_register_addr = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].address;
  uint16_t last_register_addr_length = item_ptr[dynamixel_driver_->getTheNumberOfItem(dxl_id_)-1].data_length;
  uint8_t getAllRegisteredData[last_register_addr+last_register_addr_length] = {0, };

  dynamixel_driver_->readRegister(dxl_id_, last_register_addr+last_register_addr_length, getAllRegisteredData);

  for (int index = 0; index < dynamixel_driver_->getTheNumberOfItem(dxl_id_); index++)
  {
    int32_t read_value = 0;
    read_value = getAllRegisteredData[item_ptr[index].address];

    switch (item_ptr[index].data_length)
    {
      case BYTE:
        read_value = getAllRegisteredData[item_ptr[index].address];
       break;

      case WORD:
        read_value = DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address], getAllRegisteredData[item_ptr[index].address+1]);
       break;

      case DWORD:
        read_value = DXL_MAKEDWORD(DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address],   getAllRegisteredData[item_ptr[index].address+1]),
                                   DXL_MAKEWORD(getAllRegisteredData[item_ptr[index].address+2], getAllRegisteredData[item_ptr[index].address+3]));
       break;

      default:
       read_value = getAllRegisteredData[item_ptr[index].address];
       break;
    }

    if (!strncmp(item_ptr[index].item_name, "Model_Number", strlen("Model_Number")))
      proext_state.Model_Number = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Firmware_Version", strlen("Firmware_Version")))
      proext_state.Firmware_Version = read_value;
    else if (!strncmp(item_ptr[index].item_name, "ID", strlen("ID")))
      proext_state.ID = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Baud_Rate", strlen("Baud_Rate")))
      proext_state.Baud_Rate = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Return_Delay_Time", strlen("Return_Delay_Time")))
      proext_state.Return_Delay_Time = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Operating_Mode", strlen("Operating_Mode")))
      proext_state.Operating_Mode = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Homing_Offset", strlen("Homing_Offset")))
      proext_state.Homing_Offset = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving_Threshold", strlen("Moving_Threshold")))
      proext_state.Moving_Threshold = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Temperature_Limit", strlen("Temperature_Limit")))
      proext_state.Temperature_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Voltage_Limit", strlen("Max_Voltage_Limit")))
      proext_state.Max_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Voltage_Limit", strlen("Min_Voltage_Limit")))
      proext_state.Min_Voltage_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Acceleration_Limit", strlen("Acceleration_Limit")))
      proext_state.Acceleration_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Limit", strlen("Torque_Limit")))
      proext_state.Torque_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_Limit", strlen("Velocity_Limit")))
      proext_state.Velocity_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Max_Position_Limit", strlen("Max_Position_Limit")))
      proext_state.Max_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Min_Position_Limit", strlen("Min_Position_Limit")))
      proext_state.Min_Position_Limit = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_1", strlen("External_Port_Mode_1")))
      proext_state.External_Port_Mode_1 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_2", strlen("External_Port_Mode_2")))
      proext_state.External_Port_Mode_2 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_3", strlen("External_Port_Mode_3")))
      proext_state.External_Port_Mode_3 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "External_Port_Mode_4", strlen("External_Port_Mode_4")))
      proext_state.External_Port_Mode_4 = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Shutdown", strlen("Shutdown")))
      proext_state.Shutdown = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Torque_Enable", strlen("Torque_Enable")))
      proext_state.Torque_Enable = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED_RED", strlen("LED_RED")))
      proext_state.LED_RED = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED_GREEN", strlen("LED_GREEN")))
      proext_state.LED_GREEN = read_value;
    else if (!strncmp(item_ptr[index].item_name, "LED_BLUE", strlen("LED_BLUE")))
      proext_state.LED_BLUE = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_I_Gain", strlen("Velocity_I_Gain")))
      proext_state.Velocity_I_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Velocity_P_Gain", strlen("Velocity_P_Gain")))
      proext_state.Velocity_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Position_P_Gain", strlen("Position_P_Gain")))
      proext_state.Position_P_Gain = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Position", strlen("Goal_Position")))
      proext_state.Goal_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Velocity", strlen("Goal_Velocity")))
      proext_state.Goal_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Torque", strlen("Goal_Torque")))
      proext_state.Goal_Torque = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Goal_Acceleration", strlen("Goal_Acceleration")))
      proext_state.Goal_Acceleration = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Moving", strlen("Moving")))
      proext_state.Moving = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Position", strlen("Present_Position")))
      proext_state.Present_Position = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Velocity", strlen("Present_Velocity")))
      proext_state.Present_Velocity = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Current", strlen("Present_Current")))
      proext_state.Present_Current = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Input_Voltage", strlen("Present_Input_Voltage")))
      proext_state.Present_Input_Voltage = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Present_Temperature", strlen("Present_Temperature")))
      proext_state.Present_Temperature = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Registered_Instruction", strlen("Registered_Instruction")))
      proext_state.Registered_Instruction = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Status_Return_Level", strlen("Status_Return_Level")))
      proext_state.Status_Return_Level = read_value;
    else if (!strncmp(item_ptr[index].item_name, "Hardware_Error_Status", strlen("Hardware_Error_Status")))
      proext_state.Hardware_Error_Status = read_value;
  }

  dynamixel_status_pub_.publish(proext_state);
}
