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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_QNODE_HPP
#define DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_QNODE_HPP

#ifndef Q_MOC_RUN

#include <ros/ros.h>
#include <string>
#include <QThread>
#include <QStringListModel>
#include <pthread.h>

#include "message_header.h"

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_msgs/GetDynamixelInfo.h"
#include "dynamixel_workbench_msgs/DynamixelInfo.h"

#endif

namespace qnode
{
class QNode : public QThread
{
 Q_OBJECT

 public:
  QNode(int argc, char** argv );
  virtual ~QNode();

  bool init();
  void run();

  QStringListModel* loggingModel() { return &logging_model_; }
  void log(const std::string &msg, int64_t sender);
  void log(const std::string &msg);

  // TODO : Add new Dynamixel
  void AXStatusMsgCallback(const dynamixel_workbench_msgs::AX::ConstPtr &msg);

  void RXStatusMsgCallback(const dynamixel_workbench_msgs::RX::ConstPtr &msg);

  void MXStatusMsgCallback(const dynamixel_workbench_msgs::MX::ConstPtr &msg);
  void MXExtStatusMsgCallback(const dynamixel_workbench_msgs::MXExt::ConstPtr &msg);
  void MX2StatusMsgCallback(const dynamixel_workbench_msgs::MX2::ConstPtr &msg);
  void MX2ExtStatusMsgCallback(const dynamixel_workbench_msgs::MX2Ext::ConstPtr &msg);

  void EXStatusMsgCallback(const dynamixel_workbench_msgs::EX::ConstPtr &msg);

  void XL320StatusMsgCallback(const dynamixel_workbench_msgs::XL320::ConstPtr &msg);
  void XLStatusMsgCallback(const dynamixel_workbench_msgs::XL::ConstPtr &msg);

  void XMStatusMsgCallback(const dynamixel_workbench_msgs::XM::ConstPtr &msg);
  void XMExtStatusMsgCallback(const dynamixel_workbench_msgs::XMExt::ConstPtr &msg);

  void XHStatusMsgCallback(const dynamixel_workbench_msgs::XH::ConstPtr &msg);

  void PROStatusMsgCallback(const dynamixel_workbench_msgs::PRO::ConstPtr &msg);
  void PROExtStatusMsgCallback(const dynamixel_workbench_msgs::PROExt::ConstPtr &msg);

  void AX(void);

  void RX();

  void MX();
  void MXExt();
  void MX2();
  void MX2Ext();

  void EX();

  void XL320();
  void XL();

  void XM();
  void XMExt();

  void XH();

  void PRO();
  void PROExt();

  void (qnode::QNode::*dynamixelDataLogPtr)(void);

  bool sendSetIdMsg(uint8_t set_id);
  bool sendSetOperatingModeMsg(std::string index, float protocol_version, std::string model_name, int32_t value_of_max_radian_position);
  bool sendSetBaudrateMsg(int64_t baud_rate);

  bool sendTorqueMsg(int64_t onoff);
  bool sendRebootMsg(void);
  bool sendResetMsg(void);
  bool sendAddressValueMsg(std::string addr_name, int64_t value);
  bool setPositionZeroMsg(int32_t zero_position);

  void writeReceivedDynamixelData();

  void getDynamixelInfo();
  // TODO : Add new Dynamixel
  void initDynamixelStateSubscriber();
  bool sendCommandMsg(std::string cmd, std::string addr = "", int64_t value = 0);

 Q_SIGNALS:
  void rosShutdown();

  void updateDynamixelInfo(dynamixel_workbench_msgs::DynamixelInfo);

 private:
  int init_argc;
  char** init_argv;
  pthread_mutex_t mutex;

  QStringListModel logging_model_;

  // ROS Topic Publisher

  // ROS Topic Subscriber
  ros::Subscriber dynamixel_status_msg_sub_;

  // ROS Service Server

  // ROS Service Client
  ros::ServiceClient dynamixel_info_client_;
  ros::ServiceClient dynamixel_command_client_;

  // Single Manager GUI variable
  int64_t row_count_;

  dynamixel_workbench_msgs::DynamixelInfo dynamixel_info_;
  dynamixel_workbench_msgs::AX      ax_msgs_;
  dynamixel_workbench_msgs::RX      rx_msgs_;
  dynamixel_workbench_msgs::MX      mx_msgs_;
  dynamixel_workbench_msgs::MXExt   mxext_msgs_;
  dynamixel_workbench_msgs::MX2     mx2_msgs_;
  dynamixel_workbench_msgs::MX2Ext mx2ext_msgs_;
  dynamixel_workbench_msgs::EX      ex_msgs_;
  dynamixel_workbench_msgs::XL320   xl320_msgs_;
  dynamixel_workbench_msgs::XL      xl_msgs_;
  dynamixel_workbench_msgs::XM      xm_msgs_;
  dynamixel_workbench_msgs::XMExt   xmext_msgs_;
  dynamixel_workbench_msgs::XH      xh_msgs_;
  dynamixel_workbench_msgs::PRO     pro_msgs_;
  dynamixel_workbench_msgs::PROExt  proext_msgs_;
};
}  

#endif //DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_QNODE_HPP
