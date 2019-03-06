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

#include <QMessageBox>
#include <iostream>

#include "../include/dynamixel_workbench_single_manager_gui/main_window.hpp"

using namespace main_window;
using namespace Qt;

MainWindow::MainWindow(int argc, char** argv, QWidget *parent)
    : QMainWindow(parent)
    , qnode_(argc,argv)
{
  ui_.setupUi(this);
  QObject::connect(ui_.actionAbout_Qt, SIGNAL(triggered(bool)), qApp, SLOT(aboutQt()));
  setWindowIcon(QIcon(":/images/icon.png"));

  ui_.tab_manager->setCurrentIndex(0);
  QObject::connect(&qnode_, SIGNAL(rosShutdown()), this, SLOT(close()));  

  InitConnect();

  qnode_.init();

  InitUserInterface();
}

MainWindow::~MainWindow()
{
  timer_.stop();
}

void MainWindow::showNoMasterMessage()
{
  QMessageBox msgBox;
  msgBox.setText("Couldn't find the ros master.");
  msgBox.exec();
  close();
}

void MainWindow::on_torque_enable_toggle_button_toggled(bool check)
{
  if(ui_.torque_enable_toggle_button->isChecked())  // Torque ON!!!!
  {
    ui_.torque_enable_toggle_button->setText("Torque_Disable");
    qnode_.sendTorqueMsg(true);

    ui_.set_id_combo_box->setEnabled(false);
    ui_.set_operating_mode_combo_box->setEnabled(false);
    ui_.set_baud_rate_combo_box->setEnabled(false);

    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
  }
  else                                              // Torque OFF!!!
  {
    ui_.torque_enable_toggle_button->setText("Torque_Enable");
    qnode_.sendTorqueMsg(false);

    ui_.set_id_combo_box->setEnabled(true);
    ui_.set_operating_mode_combo_box->setEnabled(true);
    ui_.set_baud_rate_combo_box->setEnabled(true);

    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
  }

  setAddressComboBox(ui_.torque_enable_toggle_button->isChecked());
}

void MainWindow::on_reboot_push_button_clicked(bool check)
{
  if (!qnode_.sendRebootMsg())
    errorMsg();
  else
    rightMsg();
}

void MainWindow::on_factory_reset_push_button_clicked(bool check)
{
  if (!qnode_.sendResetMsg())
  {
    errorMsg();
  }
  else
  {
    ui_.set_id_combo_box->setCurrentIndex(0);
    ui_.set_baud_rate_combo_box->setCurrentIndex(0);
    ui_.set_operating_mode_combo_box->setCurrentIndex(0);

    if (ui_.torque_enable_toggle_button->isChecked())
        ui_.torque_enable_toggle_button->setChecked(false);

    rightMsg();
  }
}

void MainWindow::on_set_position_zero_push_button_clicked(bool check)
{
  qnode_.setPositionZeroMsg(dynamixel_tool_->getValueOfZeroRadianPosition());
  ui_.set_address_value_spin_box->setValue(dynamixel_tool_->getValueOfZeroRadianPosition());
  ui_.set_address_value_dial->setValue(dynamixel_tool_->getValueOfZeroRadianPosition());
}

void MainWindow::changeID()
{
  if (ui_.set_id_combo_box->currentText().toStdString() != "Select ID")
  {
    if (!qnode_.sendSetIdMsg(ui_.set_id_combo_box->currentText().toInt()))
      errorMsg();
    else
    {
      rightMsg();
      ui_.get_id_line_edit->setText(QString::number(ui_.set_id_combo_box->currentText().toInt()));
    }
  }
}

void MainWindow::changeBaudrate()
{
  if (ui_.set_baud_rate_combo_box->currentText().toStdString() != "Select Baudrate")
  {
    if (!qnode_.sendSetBaudrateMsg(ui_.set_baud_rate_combo_box->currentText().toLongLong()))
      errorMsg();
    else
    {
      rightMsg();
      ui_.get_baud_rate_line_edit->setText(QString::number(ui_.set_baud_rate_combo_box->currentText().toLongLong()));
    }
  }
}

void MainWindow::changeOperatingMode()
{
  if (ui_.set_operating_mode_combo_box->currentText().toStdString() != "Select Mode")
  {
    if (!qnode_.sendSetOperatingModeMsg(ui_.set_operating_mode_combo_box->currentText().toStdString(),
                                        dynamixel_info_->lode_info.protocol_version,
                                        dynamixel_info_->model_name,
                                        dynamixel_tool_->getValueOfMaxRadianPosition()))
      errorMsg();
    else
      rightMsg();
  }
}

void MainWindow::changeControlTableValue()
{
  if (ui_.set_address_name_combo_box->currentText().toStdString() == "Torque_Enable")
  {
    if (ui_.set_address_value_spin_box->value() == true)
      ui_.torque_enable_toggle_button->setChecked(true);
    else if (ui_.set_address_value_spin_box->value() == false)
      ui_.torque_enable_toggle_button->setChecked(false);
  }

  if (!qnode_.sendAddressValueMsg(ui_.set_address_name_combo_box->currentText().toStdString(), ui_.set_address_value_dial->value()))
    errorMsg();
}

void MainWindow::setEachAddressFunction(QString index)
{
  if (index.toStdString() == "Goal_Position")
  {
    ui_.set_position_zero_push_button->setVisible(true);
    ui_.set_address_value_dial->setEnabled(true);
    ui_.set_address_value_dial->setRange(dynamixel_tool_->getValueOfMinRadianPosition(), dynamixel_tool_->getValueOfMaxRadianPosition());
    ui_.set_address_value_spin_box->setRange(dynamixel_tool_->getValueOfMinRadianPosition(), dynamixel_tool_->getValueOfMaxRadianPosition());
  }
  else
  {
    ui_.set_position_zero_push_button->setVisible(false);
    ui_.set_address_value_dial->setEnabled(false);
    ui_.set_address_value_dial->setRange(-2048, 2048);
    ui_.set_address_value_spin_box->setRange(-2048, 2048);
  }
}

void MainWindow::updateDynamixelInfoLineEdit(dynamixel_workbench_msgs::DynamixelInfo dynamixel_info)
{
  dynamixel_info_ = new DynamixelInfo;

  dynamixel_info_->lode_info.device_name      = dynamixel_info.load_info.device_name;
  dynamixel_info_->lode_info.baud_rate        = dynamixel_info.load_info.baud_rate;
  dynamixel_info_->lode_info.protocol_version = dynamixel_info.load_info.protocol_version;

  dynamixel_info_->model_id     = dynamixel_info.model_id;
  dynamixel_info_->model_number = dynamixel_info.model_number;
  dynamixel_info_->model_name   = dynamixel_info.model_name;

  dynamixel_tool_ = new DynamixelTool();
  dynamixel_tool_->addTool(dynamixel_info_->model_number, dynamixel_info_->model_id);

  ui_.get_device_name_line_edit->setText(QString::fromStdString(dynamixel_info_->lode_info.device_name));
  ui_.get_id_line_edit->setText(QString::number(dynamixel_info_->model_id));
  ui_.get_baud_rate_line_edit->setText(QString::number(dynamixel_info_->lode_info.baud_rate));
  ui_.get_protocol_version_line_edit->setText(QString::number(dynamixel_info_->lode_info.protocol_version));
  ui_.get_model_name_line_edit->setText(QString::fromStdString(dynamixel_info_->model_name));
}

void MainWindow::updateListView()
{
  qnode_.writeReceivedDynamixelData();
}

void MainWindow::InitConnect()
{
  // Init log
  ui_.view_logging->setModel(qnode_.loggingModel());

  // Init Timer
  connect(&timer_, SIGNAL(timeout()), this, SLOT(updateListView()));
  timer_.start(30);

  // Get Dynamixel Info
  qRegisterMetaType<dynamixel_workbench_msgs::DynamixelInfo>("dynamixel_workbench_msgs::DynamixelInfo");
  QObject::connect(&qnode_, SIGNAL(updateDynamixelInfo(dynamixel_workbench_msgs::DynamixelInfo)), this, SLOT(updateDynamixelInfoLineEdit(dynamixel_workbench_msgs::DynamixelInfo)));

  QObject::connect(ui_.set_id_combo_box,             SIGNAL(currentIndexChanged(int)), this, SLOT(changeID()));
  QObject::connect(ui_.set_baud_rate_combo_box,      SIGNAL(currentIndexChanged(int)), this, SLOT(changeBaudrate()));
  QObject::connect(ui_.set_operating_mode_combo_box, SIGNAL(currentIndexChanged(int)), this, SLOT(changeOperatingMode()));

  QObject::connect(ui_.set_address_value_spin_box,   SIGNAL(valueChanged(int)), ui_.set_address_value_dial, SLOT(setValue(int)));
  QObject::connect(ui_.set_address_value_dial,       SIGNAL(valueChanged(int)), ui_.set_address_value_spin_box, SLOT(setValue(int)));
  QObject::connect(ui_.set_address_value_spin_box,   SIGNAL(valueChanged(int)), this, SLOT(changeControlTableValue()));

  // Set Address function
  QObject::connect(ui_.set_address_name_combo_box,   SIGNAL(activated(QString)), this, SLOT(setEachAddressFunction(QString)));
}

void MainWindow::setIdComboBox()
{
  int id = 0;

  ui_.set_id_combo_box->addItem((QString("Select ID")));

  for (id = 1; id < 16; id++)
  {
    ui_.set_id_combo_box->addItem(QString::number(id));
  }

  // TODO  : add scroll bar

  //  QAbstractItemView* view = ui_.set_id_combo_box->view();
  //  view->setVerticalScrollBarPolicy(Qt::ScrollBarAlwaysOn);
  //  view->setAutoScroll(true);
  //  view->setFixedHeight(70);
}

void MainWindow::setBaudRateComboBox()
{
  ui_.set_baud_rate_combo_box->addItem((QString("Select Baudrate")));

  if (dynamixel_info_->lode_info.protocol_version == 1.0)
  {
    ui_.set_baud_rate_combo_box->addItem(QString::number(9600));
    ui_.set_baud_rate_combo_box->addItem(QString::number(19200));
    ui_.set_baud_rate_combo_box->addItem(QString::number(57600));
    ui_.set_baud_rate_combo_box->addItem(QString::number(115200));
    ui_.set_baud_rate_combo_box->addItem(QString::number(200000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(250000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(400000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(500000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(1000000));
  }
  else
  {
    ui_.set_baud_rate_combo_box->addItem(QString::number(9600));
    ui_.set_baud_rate_combo_box->addItem(QString::number(57600));
    ui_.set_baud_rate_combo_box->addItem(QString::number(115200));
    ui_.set_baud_rate_combo_box->addItem(QString::number(1000000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(2000000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(3000000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(4000000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(4500000));
    ui_.set_baud_rate_combo_box->addItem(QString::number(10500000));
  }
}

void MainWindow::setOperatingModeComboBox()
{
  ui_.set_operating_mode_combo_box->addItem((QString("Select Mode")));

  if (dynamixel_info_->model_name.find("AX") != std::string::npos ||
      dynamixel_info_->model_name.find("RX") != std::string::npos ||
      dynamixel_info_->model_name.find("EX") != std::string::npos)
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
  }
  else if (dynamixel_info_->model_name.find("MX") != std::string::npos)
  {
    if (dynamixel_info_->model_name.find("MX-12W") != std::string::npos ||
        dynamixel_info_->model_name.find("MX-28") != std::string::npos  ||
        dynamixel_info_->model_name.find("MX-64") != std::string::npos  ||
        dynamixel_info_->model_name.find("MX-106") != std::string::npos  )
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    }
    else if (dynamixel_info_->model_name.find("MX-28-2") != std::string::npos)
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("pwm_control"));
    }
    else if (dynamixel_info_->model_name.find("MX-64-2") != std::string::npos ||
             dynamixel_info_->model_name.find("MX-106-2") != std::string::npos)
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("current_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("current_based_position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("pwm_control"));
    }
  }
  else if (dynamixel_info_->model_name.find("XL") != std::string::npos)
  {
    if (dynamixel_info_->model_name.find("XL430-W250") != std::string::npos)
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("pwm_control"));
    }
    else if (dynamixel_info_->model_name.find("XL-320") != std::string::npos)
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    }
  }
  else if (dynamixel_info_->model_name.find("XM") != std::string::npos ||
           dynamixel_info_->model_name.find("XH") != std::string::npos)
  {
    ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("current_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("current_based_position_control"));
    ui_.set_operating_mode_combo_box->addItem(QString("pwm_control"));
  }
  else if (dynamixel_info_->model_name.find("PRO") != std::string::npos)
  {
    if (dynamixel_info_->model_name.find("PRO_L42") != std::string::npos)
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
    }
    else
    {
      ui_.set_operating_mode_combo_box->addItem(QString("position_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("velocity_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("torque_control"));
      ui_.set_operating_mode_combo_box->addItem(QString("extended_position_control"));
    }
  }
}

void MainWindow::setRebootButton()
{
  if (ui_.get_protocol_version_line_edit->text().toFloat() == 2.0)
  {
    ui_.reboot_push_button->setEnabled(true);
  }
  else
  {
    ui_.reboot_push_button->setEnabled(false);
  }
}

void MainWindow::setAddressComboBox(bool torque_enable)
{
  // Delete all Item on combo box
  uint8_t index_num = ui_.set_address_name_combo_box->count();
  for (uint8_t combo_box_index = 0; combo_box_index < index_num; combo_box_index++)
  {
    ui_.set_address_name_combo_box->removeItem(0);
  }

  uint16_t torque_enable_address = 0;
  ControlTableItem* item_ptr = dynamixel_tool_->getControlItemPtr();

  for (int item_num = 0; item_num < dynamixel_tool_->getTheNumberOfItem(); item_num++)
  {
    if (!strncmp(item_ptr[item_num].item_name, "Torque_Enable", strlen("Torque_Enable")))
    {
      torque_enable_address = item_num;
    }
  }

  // Add item on combo box
  for (int item_num = 0; item_num < dynamixel_tool_->getTheNumberOfItem(); item_num++)
  {
    if (torque_enable)
    {
      if (item_num >= torque_enable_address)
        ui_.set_address_name_combo_box->addItem(QString::fromStdString(item_ptr[item_num].item_name));
    }
    else
    {
      ui_.set_address_name_combo_box->addItem(QString::fromStdString(item_ptr[item_num].item_name));
    }
  }
}

void MainWindow::InitUserInterface()
{
  setIdComboBox();
  setBaudRateComboBox();
  setOperatingModeComboBox();
  setRebootButton();
  setAddressComboBox(false);

  ui_.set_address_value_spin_box->setRange(-2048, 2048);
  ui_.set_position_zero_push_button->setVisible(false);
  ui_.set_address_value_dial->setEnabled(false);
}

void MainWindow::on_actionAbout_triggered()
{
  QMessageBox::about(this, tr("About ..."),tr("<h2>Dynamixel workbench 1.0</h2><p>Copyright Robotis</p>"));
}

void MainWindow::errorMsg()
{
  QMessageBox::about(this, tr("Error ..."),tr("<h2>Error!!!!!</h2><p>It didn't works!!</p>"));
}

void MainWindow::rightMsg()
{
  QMessageBox::about(this, tr("Right ..."),tr("<h2>Complete!!</h2><p>It works!!</p>"));
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMainWindow::closeEvent(event);
}
