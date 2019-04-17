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

#include "gauss_driver/stepper/gauss_can_driver.h"
#include "gauss_driver/communication/can_communication.h"

GaussCanDriver::GaussCanDriver(int spi_channel, int spi_baudrate, INT8U gpio_can_interrupt, int protocol_version) 
    :protocol_version_(protocol_version)
{
    mcp_can.reset(new MCP_CAN(spi_channel, spi_baudrate, gpio_can_interrupt)); 
}

bool GaussCanDriver::setupInterruptGpio()
{
    if (!mcp_can->setupInterruptGpio()) {
        printf("Failed to start gpio");
        return CAN_GPIO_FAILINIT;
    }
    return CAN_OK;
}

bool GaussCanDriver::setupSpi()
{
    if (!mcp_can->setupSpi()) {
        printf("Failed to start spi");
        return CAN_SPI_FAILINIT;
    }
    return CAN_OK;
}

INT8U GaussCanDriver::init()
{
    // no mask or filter used, receive all messages from CAN bus
    // messages with ids != motor_id will be sent to another ROS interface
    // so we can use many CAN devices with this only driver
    int result = mcp_can->begin(MCP_ANY, CAN_1000KBPS, MCP_8MHZ);
    ROS_INFO("Result begin can : %d", result);
    
    if (result != CAN_OK) { 
        ROS_ERROR("Failed to init MCP2515 (CAN bus)");
        return result; 
    }
    
    // set mode to normal
    mcp_can->setMode(MCP_NORMAL);
    
    ros::Duration(0.05).sleep();

    if(1 == protocol_version_){
        ROS_INFO("set protocol_version: %d", protocol_version_);
        sendControlModeCommand(CAN_BROADCAST_ID, protocol_version_);
    }else if(2 == protocol_version_){
        ROS_INFO("set protocol_version: %d", protocol_version_);
        sendControlModeCommand(CAN_BROADCAST_ID, protocol_version_);        
    }
    return result;
}

//add vel command
// 2019-04-11
INT8U GaussCanDriver::sendPositionVelocityCommand(int id, int pos_cmd, int vel_cmd)
{
    if(vel_cmd <=0){
        vel_cmd = 0xFFFFFFFF;
    }
    uint8_t data[8] = { CAN_CMD_POS_VEL , 
        (uint8_t) ((pos_cmd >> 16) & 0xFF), (uint8_t) ((pos_cmd >> 8) & 0xFF), (uint8_t) (pos_cmd & 0XFF), //3 bytes
        (uint8_t) ((vel_cmd >> 24) & 0xFF), (uint8_t) ((vel_cmd >> 16) & 0xFF),
        (uint8_t) ((vel_cmd >> 8) & 0xFF), (uint8_t) (vel_cmd & 0XFF) };//4bytes
    return mcp_can->sendMsgBuf(id, 0, 8, data);
}

INT8U GaussCanDriver::sendReadPoTempCommand(int id)
{
    uint8_t data[2] = { CAN_CMD_READ, 
                    (uint8_t)(CAN_CMD_READ_POS_TEMP & 0xFF) };
    return mcp_can->sendMsgBuf(id, 0, 2, data);
}

INT8U GaussCanDriver::sendControlModeCommand(int id, int mode)
{
    uint8_t data[3] = { CAN_CMD_WRITE, 
            (uint8_t)(CAN_CMD_CONTROL_MODE & 0xFF),
            (uint8_t)(mode & 0xFF) };
    return mcp_can->sendMsgBuf(id, 0, 3, data);
}

bool GaussCanDriver::canReadData()
{
    return mcp_can->canReadData();
}

INT8U GaussCanDriver::readMsgBuf(INT32U *id, INT8U *len, INT8U *buf)
{
    return mcp_can->readMsgBuf(id, len, buf);
}

INT8U GaussCanDriver::sendPositionCommand(int id, int cmd)
{
    uint8_t data[4] = { CAN_CMD_POSITION , (uint8_t) ((cmd >> 16) & 0xFF),
        (uint8_t) ((cmd >> 8) & 0xFF), (uint8_t) (cmd & 0XFF) };
    return mcp_can->sendMsgBuf(id, 0, 4, data);
}

INT8U GaussCanDriver::sendRelativeMoveCommand(int id, int steps, int delay)
{
    uint8_t data[7] = { CAN_CMD_MOVE_REL, 
        (uint8_t) ((steps >> 16) & 0xFF), (uint8_t) ((steps >> 8) & 0xFF), (uint8_t) (steps & 0XFF),
        (uint8_t) ((delay >> 16) & 0xFF), (uint8_t) ((delay >> 8) & 0xFF), (uint8_t) (delay & 0XFF)};
    return mcp_can->sendMsgBuf(id, 0, 7, data);
}

INT8U GaussCanDriver::sendTorqueOnCommand(int id, int torque_on)
{
    uint8_t data[2] = {0};
    data[0] = CAN_CMD_MODE;
    data[1] = (torque_on) ? STEPPER_CONTROL_MODE_STANDARD : STEPPER_CONTROL_MODE_RELAX; 
    return mcp_can->sendMsgBuf(id, 0, 2, data);
}

INT8U GaussCanDriver::sendPositionOffsetCommand(int id, int cmd) 
{
    uint8_t data[4] = { CAN_CMD_OFFSET , (uint8_t) ((cmd >> 16) & 0xFF),
        (uint8_t) ((cmd >> 8) & 0xFF), (uint8_t) (cmd & 0XFF) };
    return mcp_can->sendMsgBuf(id, 0, 4, data);
}

INT8U GaussCanDriver::sendCalibrationCommand(int id, int offset, int delay, int direction, int timeout)
{
    direction = (direction == 1) ? 1 : 0;

    uint8_t data[8] = { CAN_CMD_CALIBRATE , (uint8_t) ((offset >> 16) & 0xFF),
        (uint8_t) ((offset >> 8) & 0xFF), (uint8_t) (offset & 0XFF),
        (uint8_t) ((delay >> 8) & 0xFF), (uint8_t) (delay & 0xFF), 
        (uint8_t)direction, (uint8_t)timeout };
    return mcp_can->sendMsgBuf(id, 0, 8, data);
}

INT8U GaussCanDriver::sendSynchronizePositionCommand(int id, bool begin_traj)
{
    uint8_t data[2] = { CAN_CMD_SYNCHRONIZE, (uint8_t) begin_traj };
    return mcp_can->sendMsgBuf(id, 0, 2, data);
}
   
INT8U GaussCanDriver::sendMicroStepsCommand(int id, int micro_steps)
{
    uint8_t data[2] = { CAN_CMD_MICRO_STEPS, (uint8_t) micro_steps };
    return mcp_can->sendMsgBuf(id, 0, 2, data);
}

INT8U GaussCanDriver::sendMaxEffortCommand(int id, int effort)
{
    uint8_t data[3] = { CAN_CMD_MAX_EFFORT, (uint8_t)((effort>>8) & 0xFF), (uint8_t)(effort & 0xFF)};
    return mcp_can->sendMsgBuf(id, 0, 3, data);
}
