/*
    Copyright (C) 2018 Niryo
    All rights reserved.

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include "gauss_driver/ros_interface/ros_interface.h"
#include "common/common.h"
extern GaussLogger g_roslogger_pub; 

RosInterface::RosInterface(CommunicationBase* gauss_comm, RpiDiagnostics* rpi_diagnostics,
        bool *flag_reset_controllers, bool learning_mode_on, int hardware_version)
{
    comm = gauss_comm;
    this->rpi_diagnostics = rpi_diagnostics;
    this->learning_mode_on = learning_mode_on;
    this->flag_reset_controllers = flag_reset_controllers;
    this->hardware_version = hardware_version;
    
    ros::param::get("/gauss/info/image_version", rpi_image_version);
    ros::param::get("/gauss/info/ros_version", ros_gauss_version);
   
    // trim string
    rpi_image_version.erase(rpi_image_version.find_last_not_of(" \n\r\t")+1);
    ros_gauss_version.erase(ros_gauss_version.find_last_not_of(" \n\r\t")+1);
    
    ROS_INFO("Ros interface started.");

    startServiceServers();
    startPublishers();

    // this flag is used to know if learning mode can be deactivated
    calibration_needed = 0;
}

bool RosInterface::callbackCalibrateMotors(gauss_msgs::SetInt::Request &req, gauss_msgs::SetInt::Response &res) 
{
    int calibration_mode = req.value; 
    int result = comm->allowMotorsCalibrationToStart(calibration_mode);

    res.status = 200;
    res.message = "Calibration is starting";

    // special case here 
    // we set flag learning_mode_on, but we don't activate from here
    // learning_mode should be activated in comm, AFTER motors have been calibrated
    // --> this fixes an issue where motors will jump back to a previous cmd after being calibrated
    learning_mode_on = true;
    
    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO callbackCalibrateMotors ") + res.message;
    g_roslogger_pub.publish(log_msg);
    return true;
}

bool RosInterface::callbackRequestNewCalibration(gauss_msgs::SetInt::Request &req, gauss_msgs::SetInt::Response &res)
{
    // 1. Activate learning mode
    learning_mode_on = true;
    
    comm->activateLearningMode(learning_mode_on);
    
    // publish one time
    std_msgs::Bool msg;
    msg.data = learning_mode_on;
    learning_mode_publisher.publish(msg);

    // 2. Set calibration flag (user will have to validate for calibration to start)
    comm->requestNewCalibration();

    res.status = 200;
    res.message = "New calibration request has been made, you will be requested to confirm it.";

    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO callbackRequestNewCalibration ") + res.message;
    g_roslogger_pub.publish(log_msg);
    return true;
}

/*
 * Deactivating learning mode (= activating motors) is possible only if motors are calibrated
 * Activating learning mode is also possible when waiting for calibration
 */
bool RosInterface::callbackActivateLearningMode(gauss_msgs::SetInt::Request &req, gauss_msgs::SetInt::Response &res)
{
    if (comm->isCalibrationInProgress()) {
        res.status = 400;
        res.message = "You can't activate/deactivate learning mode during motors calibration";

        std_msgs::String log_msg;
        log_msg.data = std::string("gauss_driver ERROR callbackActivateLearningMode") + res.message;
        g_roslogger_pub.publish(log_msg);
        return true;
    }

    if (calibration_needed == 1 || !comm->isConnectionOk()) { // if can or dxl is disconnected, only allow to activate learning mode
        learning_mode_on = true;
    }
    else {
        learning_mode_on = req.value; 
    }
    
    // reset controller if learning mode -> OFF
    // we want motors to start where they physically are, not from the last command
    if (!learning_mode_on) {
        *(flag_reset_controllers) = true;
        ros::Duration(0.05).sleep();
    }
    
    comm->activateLearningMode(learning_mode_on);
    
    // publish one time
    std_msgs::Bool msg;
    msg.data = learning_mode_on;
    learning_mode_publisher.publish(msg);

    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO callbackActivateLearningMode");
    g_roslogger_pub.publish(log_msg);
   
    res.status = 200;
    res.message = (learning_mode_on) ? "Activating learning mode" : "Deactivating learning mode";
    return true;
}

bool RosInterface::callbackActivateLeds(gauss_msgs::SetLeds::Request &req, gauss_msgs::SetLeds::Response &res)
{
    std::vector<int> leds = req.values;
    std::string message = "";
    bool result = comm->setLeds(leds, message);

    res.status = (result) ? 200 : 400;
    res.message = message;
    
    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO callbackActivateLeds ") + message;
    g_roslogger_pub.publish(log_msg);

    std::stringstream ss;
    ss<<leds[0]<<", ";
    ss<<leds[1]<<", ";
    ss<<leds[2]<<", ";
    ss<<leds[3]<<", ";
    ss<<"result: "<<result;
    log_msg.data = ss.str();
    g_roslogger_pub.publish(log_msg);

    return true;
}

bool RosInterface::callbackPingAndSetDxlTool(gauss_msgs::PingDxlTool::Request &req, gauss_msgs::PingDxlTool::Response &res)
{
    res.state = comm->pingAndSetDxlTool(req.id, req.name);
    
    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO callbackPingAndSetDxlTool");
    g_roslogger_pub.publish(log_msg);
    return true;
}

bool RosInterface::callbackOpenGripper(gauss_msgs::OpenGripper::Request &req, gauss_msgs::OpenGripper::Response &res)
{
    res.state = comm->openGripper(req.id, req.open_position, req.open_speed, req.open_hold_torque);

    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO callbackOpenGripper");
    g_roslogger_pub.publish(log_msg);
    return true;
}

bool RosInterface::callbackCloseGripper(gauss_msgs::CloseGripper::Request &req, gauss_msgs::CloseGripper::Response &res)
{
    res.state = comm->closeGripper(req.id, req.close_position, req.close_speed, req.close_hold_torque, req.close_max_torque);
    
    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO callbackCloseGripper");
    g_roslogger_pub.publish(log_msg);
    return true;
}

bool RosInterface::callbackPullAirVacuumPump(gauss_msgs::PullAirVacuumPump::Request &req, gauss_msgs::PullAirVacuumPump::Response &res)
{
    res.state = comm->pullAirVacuumPump(req.id, req.pull_air_position, req.pull_air_hold_torque);
    
    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO pullAirVacuumPump");
    g_roslogger_pub.publish(log_msg);
    return true;
}

bool RosInterface::callbackPushAirVacuumPump(gauss_msgs::PushAirVacuumPump::Request &req, gauss_msgs::PushAirVacuumPump::Response &res)
{
    res.state = comm->pushAirVacuumPump(req.id, req.push_air_position);
    
    std_msgs::String log_msg;
    log_msg.data = std::string("gauss_driver INFO pushAirVacuumPump");
    g_roslogger_pub.publish(log_msg);
    return true;
}

bool RosInterface::callbackChangeHardwareVersion(gauss_msgs::ChangeHardwareVersion::Request &req,
        gauss_msgs::ChangeHardwareVersion::Response &res)
{
    int result = change_hardware_version_and_reboot(req.old_version, req.new_version);
    if (result == CHANGE_HW_VERSION_OK) {
        res.status = 200;
        res.message = "Successfully changed hardware version.";

        std_msgs::String log_msg;
        log_msg.data = std::string("gauss_driver INFO callbackChangeHardwareVersion ") + res.message;
        g_roslogger_pub.publish(log_msg);
    }
    else if (result == CHANGE_HW_VERSION_FAIL) {
        res.status = 400;
        res.message = "Failed to change hardware version, please check the ROS logs";

        std_msgs::String log_msg;
        log_msg.data = std::string("gauss_driver ERROR callbackChangeHardwareVersion ") + res.message;
        g_roslogger_pub.publish(log_msg);
    }
    else if (result == CHANGE_HW_VERSION_NOT_ARM) {
        res.status = 400;
        res.message = "Not allowed to change hardware version on non-ARM system";

        std_msgs::String log_msg;
        log_msg.data = std::string("gauss_driver ERROR callbackChangeHardwareVersion ") + res.message;
        g_roslogger_pub.publish(log_msg);
    }
    return true;
}

void RosInterface::startServiceServers()
{
    calibrate_motors_server = nh_.advertiseService("gauss/calibrate_motors", &RosInterface::callbackCalibrateMotors, this);
    request_new_calibration_server = nh_.advertiseService("gauss/request_new_calibration", &RosInterface::callbackRequestNewCalibration, this);

    activate_learning_mode_server = nh_.advertiseService("gauss/activate_learning_mode", &RosInterface::callbackActivateLearningMode, this);
    activate_leds_server = nh_.advertiseService("gauss/set_dxl_leds", &RosInterface::callbackActivateLeds, this);

    ping_and_set_dxl_tool_server = nh_.advertiseService("gauss/tools/ping_and_set_dxl_tool", &RosInterface::callbackPingAndSetDxlTool, this);
    open_gripper_server = nh_.advertiseService("gauss/tools/open_gripper", &RosInterface::callbackOpenGripper, this);
    close_gripper_server = nh_.advertiseService("gauss/tools/close_gripper", &RosInterface::callbackCloseGripper, this);
    pull_air_vacuum_pump_server = nh_.advertiseService("gauss/tools/pull_air_vacuum_pump", &RosInterface::callbackPullAirVacuumPump, this);
    push_air_vacuum_pump_server = nh_.advertiseService("gauss/tools/push_air_vacuum_pump", &RosInterface::callbackPushAirVacuumPump, this);

    change_hardware_version_server = nh_.advertiseService("gauss/change_hardware_version", &RosInterface::callbackChangeHardwareVersion, this);
}

void RosInterface::publishHardwareStatus()
{
    double publish_hw_status_frequency;
    ros::param::get("~publish_hw_status_frequency", publish_hw_status_frequency);
    ros::Rate publish_hardware_status_rate = ros::Rate(publish_hw_status_frequency);

    while (ros::ok()) {
        ros::Time time_now = ros::Time::now();

        bool connection_up = false;
        bool calibration_in_progress = false;
        std::string error_message;
        std::vector<std::string> motor_names;
        std::vector<std::string> motor_types;
        std::vector<int32_t> temperatures;
        std::vector<double> voltages;
        std::vector<int32_t> hw_errors;

        comm->getHardwareStatus(&connection_up, error_message, &calibration_needed, 
                &calibration_in_progress, motor_names, motor_types, temperatures, voltages, hw_errors);

        gauss_msgs::HardwareStatus msg;
        msg.header.stamp = ros::Time::now();
        msg.rpi_temperature = rpi_diagnostics->getRpiCpuTemperature();
        msg.hardware_version = hardware_version;
        msg.connection_up = connection_up;
        msg.error_message = error_message;
        msg.calibration_needed = calibration_needed;
        msg.calibration_in_progress = calibration_in_progress;
        msg.motor_names = motor_names;
        msg.motor_types = motor_types;
        msg.temperatures = temperatures;
        msg.voltages = voltages;
        msg.hardware_errors = hw_errors;

        hardware_status_publisher.publish(msg);

        publish_hardware_status_rate.sleep();
    }
}

void RosInterface::publishSoftwareVersion()
{
    double publish_software_version_frequency;
    ros::param::get("~publish_software_version_frequency", publish_software_version_frequency);
    ros::Rate publish_software_version_rate = ros::Rate(publish_software_version_frequency);

    while (ros::ok()) {
        std::vector<std::string> motor_names;
        std::vector<std::string> firmware_versions;
        comm->getFirmwareVersions(motor_names, firmware_versions);

        gauss_msgs::SoftwareVersion msg;
        msg.motor_names = motor_names;
        msg.stepper_firmware_versions = firmware_versions;
        msg.rpi_image_version = rpi_image_version;
        msg.ros_gauss_version = ros_gauss_version;
       
        software_version_publisher.publish(msg);
        publish_software_version_rate.sleep();
    }
}

void RosInterface::publishLearningMode()
{
    double publish_learning_mode_frequency;
    ros::param::get("~publish_learning_mode_frequency", publish_learning_mode_frequency);
    ros::Rate publish_learning_mode_rate = ros::Rate(publish_learning_mode_frequency);

    while (ros::ok()) {
        std_msgs::Bool msg;
        msg.data = learning_mode_on;
        learning_mode_publisher.publish(msg);
        publish_learning_mode_rate.sleep();
    }
}

void RosInterface::startPublishers()
{
    hardware_status_publisher = nh_.advertise<gauss_msgs::HardwareStatus>("gauss/hardware_status", 10);
    publish_hardware_status_thread.reset(new std::thread(boost::bind(&RosInterface::publishHardwareStatus, this))); 

    software_version_publisher = nh_.advertise<gauss_msgs::SoftwareVersion>("gauss/software_version", 10);
    publish_software_version_thread.reset(new std::thread(boost::bind(&RosInterface::publishSoftwareVersion, this)));

    learning_mode_publisher = nh_.advertise<std_msgs::Bool>("gauss/learning_mode", 10);
    publish_learning_mode_thread.reset(new std::thread(boost::bind(&RosInterface::publishLearningMode, this)));
}



