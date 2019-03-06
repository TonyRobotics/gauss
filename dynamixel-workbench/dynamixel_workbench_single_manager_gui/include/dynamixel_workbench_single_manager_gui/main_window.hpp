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

#ifndef DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_MAIN_WINDOW_H
#define DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_MAIN_WINDOW_H

#ifndef Q_MOC_RUN

#include <QMainWindow>
#include <QTimer>
#include "ui_main_window.h"
#include "qnode.hpp"

#include "dynamixel_workbench_msgs/DynamixelCommand.h"
#include "dynamixel_workbench_toolbox/dynamixel_driver.h"

#endif

namespace main_window
{

struct DynamixelLoadInfo
{
  std::string device_name;
  int baud_rate;
  float protocol_version;
};

struct DynamixelInfo
{
  int16_t model_number;
  int8_t model_id;
  std::string model_name;
  DynamixelLoadInfo lode_info;
};

class MainWindow : public QMainWindow
{
 Q_OBJECT

 public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();

  void closeEvent(QCloseEvent *event);
	void showNoMasterMessage();

 public Q_SLOTS:
  void on_actionAbout_triggered();
  void on_torque_enable_toggle_button_toggled(bool check);
  void on_reboot_push_button_clicked(bool check);
  void on_factory_reset_push_button_clicked(bool check);
  void on_set_position_zero_push_button_clicked(bool check);

  void changeID();
  void changeBaudrate();
  void changeOperatingMode();
  void changeControlTableValue();
  void setEachAddressFunction(QString index);

  void updateDynamixelInfoLineEdit(dynamixel_workbench_msgs::DynamixelInfo dynamixel_info);
  void updateListView();

 private:
  Ui::MainWindowDesign ui_;
  qnode::QNode qnode_;
  QTimer timer_;

  DynamixelInfo *dynamixel_info_;
  DynamixelTool *dynamixel_tool_;

  bool reboot_button_;
  bool operating_mode_spinbox_;

  void InitUserInterface();
  void InitConnect();

  void setIdComboBox();
  void setBaudRateComboBox();
  void setOperatingModeComboBox();
  void setRebootButton();
  void setAddressComboBox(bool torque_enable);

  void errorMsg();
  void rightMsg();
};
}

#endif // DYNAMIXEL_WORKBENCH_SINGLE_MANAGER_GUI_MAIN_WINDOW_H
