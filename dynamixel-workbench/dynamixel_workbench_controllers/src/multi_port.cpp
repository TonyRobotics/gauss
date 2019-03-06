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

#include "dynamixel_workbench_controllers/multi_port.h"

MultiPort::MultiPort()
    :node_handle_("")
{
  std::string device_name[2];
  uint32_t dxl_baud_rate[2];

  device_name[FIRST]    = node_handle_.param<std::string>("pan/device_name", "/dev/ttyUSB0");
  dxl_baud_rate[FIRST]  = node_handle_.param<int>("pan/baud_rate", 57600);

  device_name[SECOND]   = node_handle_.param<std::string>("tilt/device_name", "/dev/ttyUSB1");
  dxl_baud_rate[SECOND] = node_handle_.param<int>("tilt/baud_rate", 57600);

  uint8_t scan_range            = node_handle_.param<int>("scan_range", 200);

  uint32_t profile_velocity     = node_handle_.param<int>("profile_velocity", 200);
  uint32_t profile_acceleration = node_handle_.param<int>("profile_acceleration", 50);

  dxl_wb_[FIRST]  = new DynamixelWorkbench;
  dxl_wb_[SECOND] = new DynamixelWorkbench;

  for (int port_cnt = 0; port_cnt < PORT_NUM; port_cnt++)
  {
    dxl_wb_[port_cnt]->begin(device_name[port_cnt].c_str(), dxl_baud_rate[port_cnt]);
    if (dxl_wb_[port_cnt]->scan(dxl_id_[port_cnt], &dxl_cnt_[port_cnt], scan_range) != true)
    {
      ROS_ERROR("Not found Motors, Please check scan range and baud rate");
      ros::shutdown();
      return;
    }
  }
  initMsg();

  for (int port_cnt = 0; port_cnt < PORT_NUM; port_cnt++)
  {
    for (int index = 0; index < dxl_cnt_[port_cnt]; index++)
      dxl_wb_[port_cnt]->jointMode(dxl_id_[port_cnt][index], profile_velocity, profile_acceleration);
  }

  initPublisher();
  initServer();
}

MultiPort::~MultiPort()
{
  for (int port_cnt = 0; port_cnt < PORT_NUM; port_cnt++)
  {
    for (int index = 0; index < dxl_cnt_[port_cnt]; index++)
      dxl_wb_[port_cnt]->itemWrite(dxl_id_[port_cnt][index], "Torque_Enable", 0);
  }

  ros::shutdown();
}

void MultiPort::initMsg()
{
  printf("-----------------------------------------------------------------------\n");
  printf("  dynamixel_workbench controller; multi port example                    \n");
  printf("-----------------------------------------------------------------------\n");
  printf("\n");

  for (int port_cnt = 0; port_cnt < PORT_NUM; port_cnt++)
  {
    for (int index = 0; index < dxl_cnt_[port_cnt]; index++)
    {
      printf("MODEL   : %s\n", dxl_wb_[port_cnt]->getModelName(dxl_id_[port_cnt][index]));
      printf("ID      : %d\n", dxl_id_[port_cnt][index]);
      printf("\n");
    }
    printf("-----------------------------------------------------------------------\n\n");
  }
}

void MultiPort::initPublisher()
{
  dynamixel_state_list_pub_ = node_handle_.advertise<dynamixel_workbench_msgs::DynamixelStateList>("dynamixel_state", 10);
}

void MultiPort::initServer()
{
  joint_command_server_ = node_handle_.advertiseService("joint_command", &MultiPort::jointCommandMsgCallback, this);
}

void MultiPort::dynamixelStatePublish()
{
  uint8_t dxl_num = dxl_cnt_[FIRST] + dxl_cnt_[SECOND];
  uint8_t cnt = 0;

  dynamixel_workbench_msgs::DynamixelState     dynamixel_state[dxl_num];
  dynamixel_workbench_msgs::DynamixelStateList dynamixel_state_list;

  for (int port_cnt = 0; port_cnt < PORT_NUM; port_cnt++)
  {
    for (int index = 0; index < dxl_cnt_[port_cnt]; index++)
    {
      dynamixel_state[cnt].model_name          = std::string(dxl_wb_[port_cnt]->getModelName(dxl_id_[port_cnt][index]));
      dynamixel_state[cnt].id                  = dxl_id_[port_cnt][index];
      dynamixel_state[cnt].torque_enable       = dxl_wb_[port_cnt]->itemRead(dxl_id_[port_cnt][index], "Torque_Enable");
      dynamixel_state[cnt].present_position    = dxl_wb_[port_cnt]->itemRead(dxl_id_[port_cnt][index], "Present_Position");
      dynamixel_state[cnt].present_velocity    = dxl_wb_[port_cnt]->itemRead(dxl_id_[port_cnt][index], "Present_Velocity");
      dynamixel_state[cnt].goal_position       = dxl_wb_[port_cnt]->itemRead(dxl_id_[port_cnt][index], "Goal_Position");
      dynamixel_state[cnt].goal_velocity       = dxl_wb_[port_cnt]->itemRead(dxl_id_[port_cnt][index], "Goal_Velocity");
      dynamixel_state[cnt].moving              = dxl_wb_[port_cnt]->itemRead(dxl_id_[port_cnt][index], "Moving");

      dynamixel_state_list.dynamixel_state.push_back(dynamixel_state[cnt]);
      cnt++;
    }
  }
  dynamixel_state_list_pub_.publish(dynamixel_state_list);
}

void MultiPort::controlLoop()
{
  dynamixelStatePublish();
}

bool MultiPort::jointCommandMsgCallback(dynamixel_workbench_msgs::JointCommand::Request &req,
                                              dynamixel_workbench_msgs::JointCommand::Response &res)
{
  int32_t goal_position = 0;
  int32_t present_position = 0;

  uint8_t port = 0;

  for (int port_cnt = 0; port_cnt < PORT_NUM; port_cnt++)
  {
    for (int index = 0; index < dxl_cnt_[port_cnt]; index++)
    {
      if (dxl_id_[port_cnt][index] == req.id)
        port = port_cnt;
    }
  }

  if (req.unit == "rad")
  {
    goal_position = dxl_wb_[port]->convertRadian2Value(req.id, req.goal_position);
  }
  else if (req.unit == "raw")
  {
    goal_position = req.goal_position;
  }
  else
  {
    goal_position = req.goal_position;
  }

  bool ret = dxl_wb_[port]->goalPosition(req.id, goal_position);

  res.result = ret;
}

int main(int argc, char **argv)
{
  // Init ROS node
  ros::init(argc, argv, "multi_port");
  MultiPort multi;

  ros::Rate loop_rate(250);

  while (ros::ok())
  {
    multi.controlLoop();
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}
