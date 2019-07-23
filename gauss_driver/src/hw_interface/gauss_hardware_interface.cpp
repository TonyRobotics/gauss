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

#include "gauss_driver/hw_interface/gauss_hardware_interface.h"

GaussHardwareInterface::GaussHardwareInterface(CommunicationBase* gauss_comm) 
{
    comm = gauss_comm;
    ROS_INFO("Starting Gauss Hardware Interface...");

    // connect and register joint state interface
    hardware_interface::JointStateHandle state_handle1("joint1", &pos[0], &vel[0], &eff[0]);
    joint_state_interface.registerHandle(state_handle1);
    hardware_interface::JointStateHandle state_handle2("joint2", &pos[1], &vel[1], &eff[1]);
    joint_state_interface.registerHandle(state_handle2);
    hardware_interface::JointStateHandle state_handle3("joint3", &pos[2], &vel[2], &eff[2]);
    joint_state_interface.registerHandle(state_handle3);
    hardware_interface::JointStateHandle state_handle4("joint4", &pos[3], &vel[3], &eff[3]);
    joint_state_interface.registerHandle(state_handle4);
    hardware_interface::JointStateHandle state_handle5("joint5", &pos[4], &vel[4], &eff[4]);
    joint_state_interface.registerHandle(state_handle5);
    hardware_interface::JointStateHandle state_handle6("joint6", &pos[5], &vel[5], &eff[5]);
    joint_state_interface.registerHandle(state_handle6);

    registerInterface(&joint_state_interface);

    // connect and register joint position interface
    hardware_interface::JointHandle position_handle1(joint_state_interface.getHandle("joint1"), &cmd[0]);
    joint_position_interface.registerHandle(position_handle1);
    hardware_interface::JointHandle position_handle2(joint_state_interface.getHandle("joint2"), &cmd[1]);
    joint_position_interface.registerHandle(position_handle2);
    hardware_interface::JointHandle position_handle3(joint_state_interface.getHandle("joint3"), &cmd[2]);
    joint_position_interface.registerHandle(position_handle3);
    hardware_interface::JointHandle position_handle4(joint_state_interface.getHandle("joint4"), &cmd[3]);
    joint_position_interface.registerHandle(position_handle4);
    hardware_interface::JointHandle position_handle5(joint_state_interface.getHandle("joint5"), &cmd[4]);
    joint_position_interface.registerHandle(position_handle5);
    hardware_interface::JointHandle position_handle6(joint_state_interface.getHandle("joint6"), &cmd[5]);
    joint_position_interface.registerHandle(position_handle6);

    registerInterface(&joint_position_interface);

    ROS_INFO("Interfaces registered.");

    pos_cmd_queues_.push_back(&joint1_pos_cmd_queue_);
    pos_cmd_queues_.push_back(&joint2_pos_cmd_queue_);
    pos_cmd_queues_.push_back(&joint3_pos_cmd_queue_);
    pos_cmd_queues_.push_back(&joint4_pos_cmd_queue_);
    pos_cmd_queues_.push_back(&joint5_pos_cmd_queue_);
    pos_cmd_queues_.push_back(&joint6_pos_cmd_queue_);

    ros::param::get("~data_control_loop_frequency", data_control_loop_frequency_);
    data_control_loop_thread.reset(new std::thread(boost::bind(&GaussHardwareInterface::dataControlLoop, this)));

}

void GaussHardwareInterface::setCommandToCurrentPosition()
{
    joint_position_interface.getHandle("joint1").setCommand(pos[0]);
    joint_position_interface.getHandle("joint2").setCommand(pos[1]);
    joint_position_interface.getHandle("joint3").setCommand(pos[2]);
    joint_position_interface.getHandle("joint4").setCommand(pos[3]);
    joint_position_interface.getHandle("joint5").setCommand(pos[4]);
    joint_position_interface.getHandle("joint6").setCommand(pos[5]);
}

std::vector<double> GaussHardwareInterface::getPositionToWrite()
{
    std::vector<double> ret;
    for(int i = 0; i < 6; i++)
    {
        ret.push_back(pos[i]); 
    }
    return ret;
}

void GaussHardwareInterface::read()
{
    //ROS_INFO("Read sensor values");
    
    double pos_to_read[6] = {0.0};
    
    comm->getCurrentPosition(pos_to_read);
   
    pos[0] = pos_to_read[0];
    pos[1] = pos_to_read[1];
    pos[2] = pos_to_read[2];
    pos[3] = pos_to_read[3];
    pos[4] = pos_to_read[4];
    pos[5] = pos_to_read[5];
}

void GaussHardwareInterface::write()
{
    // for debugging
    //pos[0] = cmd[0];
    //pos[1] = cmd[1];
    //pos[2] = cmd[2];
    //pos[3] = cmd[3];
    //pos[4] = cmd[4];
    //pos[5] = cmd[5];

    // comm->sendPositionToRobot(cmd);
    data_mutex_.lock();
    for(size_t i = 0; i < 6; i++)
    {
        if( pos_cmd_queues_.at(i)->size() !=0 ){         
            if(fabs(cmd[i] - pos_cmd_queues_.at(i)->back()) <= 1e-8){
                continue;
            }
        }
        pos_cmd_queues_.at(i)->push(cmd[i]);
    }
    data_mutex_.unlock();
}


void GaussHardwareInterface::dataControlLoop()
{
    ROS_INFO("thread dataControlLoop");

    ros::Rate data_control_loop_frequency_rate = ros::Rate(data_control_loop_frequency_); 

    while (ros::ok()) {
       data_mutex_.lock();

       double pos_cmd_tmp[6];
       bool got_data_flag = false;

       for(size_t i = 0; i < 6; i++)
        {
            if(!pos_cmd_queues_.at(i)->size()){
                continue;
            }else{
                got_data_flag = true;
            }

            double cmd = pos_cmd_queues_.at(i)->front();
            // printf("--------pos queue size %d, cmd %f\n",pos_cmd_queues_.at(i)->size(), cmd);
            pos_cmd_tmp[i] = cmd;
        } 

        if(got_data_flag){
            if(comm->sendPositionToRobot(pos_cmd_tmp)){ //send success
                for(size_t i = 0; i < 6; i++)
                {
                    pos_cmd_queues_.at(i)->pop();                
                }
                // std::cout<<"---------- send success"<<std::endl;
            }else{
                // std::cout<<"---------- wait to send"<<std::endl;
            }
        }

        data_mutex_.unlock();
        
        data_control_loop_frequency_rate.sleep();      
    }
}