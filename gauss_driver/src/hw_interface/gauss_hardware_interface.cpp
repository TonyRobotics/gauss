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
    // hardware_interface::JointHandle position_handle1(joint_state_interface.getHandle("joint1"), &cmd[0]);
    // joint_position_interface.registerHandle(position_handle1);
    // hardware_interface::JointHandle position_handle2(joint_state_interface.getHandle("joint2"), &cmd[1]);
    // joint_position_interface.registerHandle(position_handle2);
    // hardware_interface::JointHandle position_handle3(joint_state_interface.getHandle("joint3"), &cmd[2]);
    // joint_position_interface.registerHandle(position_handle3);
    // hardware_interface::JointHandle position_handle4(joint_state_interface.getHandle("joint4"), &cmd[3]);
    // joint_position_interface.registerHandle(position_handle4);
    // hardware_interface::JointHandle position_handle5(joint_state_interface.getHandle("joint5"), &cmd[4]);
    // joint_position_interface.registerHandle(position_handle5);
    // hardware_interface::JointHandle position_handle6(joint_state_interface.getHandle("joint6"), &cmd[5]);
    // joint_position_interface.registerHandle(position_handle6);

    // registerInterface(&joint_position_interface);

   // connect and register joint position and vel interface
    hardware_interface::PosVelJointHandle position_handle1(joint_state_interface.getHandle("joint1"), &pos_cmd[0], &vel_cmd[0]);
    pos_vel_joint_interface.registerHandle(position_handle1);
    hardware_interface::PosVelJointHandle position_handle2(joint_state_interface.getHandle("joint2"), &pos_cmd[1], &vel_cmd[1]);
    pos_vel_joint_interface.registerHandle(position_handle2);
    hardware_interface::PosVelJointHandle position_handle3(joint_state_interface.getHandle("joint3"), &pos_cmd[2], &vel_cmd[2]);
    pos_vel_joint_interface.registerHandle(position_handle3);
    hardware_interface::PosVelJointHandle position_handle4(joint_state_interface.getHandle("joint4"), &pos_cmd[3], &vel_cmd[3]);
    pos_vel_joint_interface.registerHandle(position_handle4);
    hardware_interface::PosVelJointHandle position_handle5(joint_state_interface.getHandle("joint5"), &pos_cmd[4], &vel_cmd[4]);
    pos_vel_joint_interface.registerHandle(position_handle5);
    hardware_interface::PosVelJointHandle position_handle6(joint_state_interface.getHandle("joint6"), &pos_cmd[5], &vel_cmd[5]);
    pos_vel_joint_interface.registerHandle(position_handle6);

    registerInterface(&pos_vel_joint_interface);
    
    ros::param::get("~data_control_loop_frequency", data_control_loop_frequency_);

    int can_protocol_version = -1;
    ros::param::get("/gauss/can_protocol_version", can_protocol_version);
    if(2 == can_protocol_version){

       pos_cmd_queues_.push_back(&joint1_pos_cmd_queue_);
       pos_cmd_queues_.push_back(&joint2_pos_cmd_queue_);
       pos_cmd_queues_.push_back(&joint3_pos_cmd_queue_);
       pos_cmd_queues_.push_back(&joint4_pos_cmd_queue_);
       pos_cmd_queues_.push_back(&joint5_pos_cmd_queue_);
       pos_cmd_queues_.push_back(&joint6_pos_cmd_queue_);

       vel_cmd_queues_.push_back(&joint1_vel_cmd_queue_);
       vel_cmd_queues_.push_back(&joint2_vel_cmd_queue_);
       vel_cmd_queues_.push_back(&joint3_vel_cmd_queue_);
       vel_cmd_queues_.push_back(&joint4_vel_cmd_queue_);
       vel_cmd_queues_.push_back(&joint5_vel_cmd_queue_);
       vel_cmd_queues_.push_back(&joint6_vel_cmd_queue_);

        ROS_INFO("vel_cmd_queues_ init.");

        data_control_loop_thread.reset(new std::thread(boost::bind(&GaussHardwareInterface::dataControlLoop, this)));
    }

    ROS_INFO("Interfaces registered.");
}

// void GaussHardwareInterface::setCommandToCurrentPosition()
// {
//     joint_position_interface.getHandle("joint1").setCommand(pos[0]);
//     joint_position_interface.getHandle("joint2").setCommand(pos[1]);
//     joint_position_interface.getHandle("joint3").setCommand(pos[2]);
//     joint_position_interface.getHandle("joint4").setCommand(pos[3]);
//     joint_position_interface.getHandle("joint5").setCommand(pos[4]);
//     joint_position_interface.getHandle("joint6").setCommand(pos[5]);
// }

void GaussHardwareInterface::dataControlLoop()
{
    ROS_INFO("thread dataControlLoop");

    ros::Rate data_control_loop_frequency_rate = ros::Rate(data_control_loop_frequency_); 

    while (ros::ok()) {
        data_mutex_.lock();

       double pos_cmd[6];
       double vel_cmd[6];
       for(size_t i = 0; i < 6; i++)
        {
            if(!pos_cmd_queues_.at(i)->size())
                continue;

            double cmd = pos_cmd_queues_.at(i)->front();
            //printf("--------pos queue size %d, cmd %f\n",pos_cmd_queues_.at(i)->size(), cmd);
            pos_cmd[i] = cmd;
            pos_cmd_queues_.at(i)->pop();
        }  
       
       for(size_t i = 0; i < 6; i++)
        {
            if(!vel_cmd_queues_.at(i)->size())
                continue;

            double cmd = vel_cmd_queues_.at(i)->front();
            //printf("--------vel queue size %d, cmd %f\n",vel_cmd_queues_.at(i)->size(), cmd);
            vel_cmd[i] = cmd;
            vel_cmd_queues_.at(i)->pop();
        }  

        data_mutex_.unlock();

        comm->sendPositionVelocityToRobot(pos_cmd, vel_cmd);  
        // for(size_t i = 0; i < 6; i++)
        // {
            // printf("--------%d pos_cmd %f", i, pos_cmd[i]);
            // printf(" vel_cmd %f\n", vel_cmd[i]);
        // }        

        data_control_loop_frequency_rate.sleep();      
    }
}

void GaussHardwareInterface::setCommandToCurrentPosition()
{
    if(1 == comm->getCANProtocolVersion()){
        pos_vel_joint_interface.getHandle("joint1").setCommandPosition(pos[0]);
        pos_vel_joint_interface.getHandle("joint2").setCommandPosition(pos[1]);
        pos_vel_joint_interface.getHandle("joint3").setCommandPosition(pos[2]);
        pos_vel_joint_interface.getHandle("joint4").setCommandPosition(pos[3]);
        pos_vel_joint_interface.getHandle("joint5").setCommandPosition(pos[4]);
        pos_vel_joint_interface.getHandle("joint6").setCommandPosition(pos[5]);

    }else if(2 == comm->getCANProtocolVersion() ){
        pos_vel_joint_interface.getHandle("joint1").setCommandPosition(pos[0]);
        pos_vel_joint_interface.getHandle("joint2").setCommandPosition(pos[1]);
        pos_vel_joint_interface.getHandle("joint3").setCommandPosition(pos[2]);
        pos_vel_joint_interface.getHandle("joint4").setCommandPosition(pos[3]);
        pos_vel_joint_interface.getHandle("joint5").setCommandPosition(pos[4]);
        pos_vel_joint_interface.getHandle("joint6").setCommandPosition(pos[5]);

        pos_vel_joint_interface.getHandle("joint1").setCommandVelocity(DEFAULT_LOW_SPEED);
        pos_vel_joint_interface.getHandle("joint2").setCommandVelocity(DEFAULT_LOW_SPEED);
        pos_vel_joint_interface.getHandle("joint3").setCommandVelocity(DEFAULT_LOW_SPEED);
        pos_vel_joint_interface.getHandle("joint4").setCommandVelocity(DEFAULT_LOW_SPEED);
        pos_vel_joint_interface.getHandle("joint5").setCommandVelocity(DEFAULT_LOW_SPEED);
        pos_vel_joint_interface.getHandle("joint6").setCommandVelocity(DEFAULT_LOW_SPEED);
    }
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
    if(1 == comm->getCANProtocolVersion()){
        comm->getCurrentPosition(pos); 
    }else if(2 == comm->getCANProtocolVersion() ){         
        comm->getCurrentPosTemp(pos, temp);
        // std::cout<<"~~~~~~~~~~~~~~~~ read"<<std::endl;
        // std::cout<<"getCurrentPosTemp read: "<< pos[0]<<std::endl;
        // std::cout<<"getCurrentPosTemp read: "<< pos[1]<<std::endl;
        // std::cout<<"getCurrentPosTemp read: "<< pos[2]<<std::endl;
        // std::cout<<"getCurrentPosTemp read: "<< pos[3]<<std::endl;
     }
}

void GaussHardwareInterface::write()
{
    // std::cout<<"comm->getCANProtocolVersion() "<< comm->getCANProtocolVersion()<<std::endl;
     if(1 == comm->getCANProtocolVersion()){
        comm->sendPositionToRobot(pos_cmd);
    }else if(2 == comm->getCANProtocolVersion()){
        // comm->sendPositionVelocityToRobot(pos_cmd, vel_cmd);  
        // std::cout<<"sendPositionVelocityToRobot write -- : "<< vel_cmd[2]<<std::endl;
        // for(size_t i = 0; i < 6; i++)
        // {
            // std::cout<<"GaussHardwareInterface -- got i: "<< i <<std::endl;
            // std::cout<<"GaussHardwareInterface -- got pos_cmd: " <<pos_cmd[i]<< " vel_cmd: "<< vel_cmd[i]<<std::endl;
        // }
        
        data_mutex_.lock();

        for(size_t i = 0; i < 6; i++)
        {
            if( pos_cmd_queues_.at(i)->size() !=0 && vel_cmd_queues_.at(i)->size() !=0){
                if(fabs(vel_cmd_queues_.at(i)->back()) <= 1e-8){
                    if( fabs(pos_cmd[i] - pos_cmd_queues_.at(i)->back()) <= 1e-8  &&  fabs(vel_cmd[i]) <= 1e-8 ){
                        continue;
                    }
                }
            } 
            pos_cmd_queues_.at(i)->push(pos_cmd[i]);       
            vel_cmd_queues_.at(i)->push(vel_cmd[i]);
        }  
        data_mutex_.unlock();
    }
}

