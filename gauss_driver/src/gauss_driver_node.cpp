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

#include <boost/shared_ptr.hpp>
#include <controller_manager/controller_manager.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <vector>
#include <sstream>
#include <string>
#include <thread>

#include "gauss_driver/hw_interface/gauss_hardware_interface.h"
#include "gauss_driver/communication/communication_base.h"
#include "gauss_driver/communication/gauss_communication.h"
#include "gauss_driver/communication/fake_communication.h"
#include "gauss_driver/ros_interface/ros_interface.h"
#include "gauss_driver/rpi/rpi_diagnostics.h"

#include "control_msgs/FollowJointTrajectoryActionResult.h"
#include "std_msgs/Empty.h"
#include "std_msgs/String.h"
#include "std_msgs/Bool.h"

#include "common/common.h"

bool g_debug_mode = false;

GaussLogger g_roslogger_pub;

static void timespecInc(struct timespec &tick, int nsec)
{
  int SEC_2_NSEC = 1e+9;
  tick.tv_nsec += nsec;
  while (tick.tv_nsec >= SEC_2_NSEC)
  {
    tick.tv_nsec -= SEC_2_NSEC;
    ++tick.tv_sec;
  }
}

class GaussDriver {

    private:
    
    int hardware_version;
    int can_protocol_version_;

    boost::shared_ptr<CommunicationBase> comm;

    boost::shared_ptr<GaussHardwareInterface> robot;
    boost::shared_ptr<controller_manager::ControllerManager> cm;

    boost::shared_ptr<RosInterface> ros_interface;

    boost::shared_ptr<RpiDiagnostics> rpi_diagnostics;

    boost::shared_ptr<ros::Rate> ros_control_loop_rate;

    boost::shared_ptr<std::thread> ros_control_thread;

    ros::NodeHandle nh_;

    bool flag_reset_controllers;

    ros::Subscriber reset_controller_subscriber; // workaround to compensate missed steps
    ros::Subscriber debug_mode_subscriber;
    ros::Subscriber trajectory_result_subscriber;

    bool debug_mode;

    public:

    // void rosControlLoop() 
    // {
    //     ros::Time last_time = ros::Time::now();
    //     ros::Time current_time = ros::Time::now();
    //     ros::Duration elapsed_time;
        
    //     while(ros::ok()) {
        
    //       robot->read();
    //       current_time = ros::Time::now();
    //       elapsed_time = ros::Duration(current_time - last_time);
    //       last_time = current_time;        

    //       if (flag_reset_controllers) {
    //         robot->setCommandToCurrentPosition();
    //         cm->update(ros::Time::now(), elapsed_time, true);
    //         flag_reset_controllers = false;
    //       }
    //       else {
    //         cm->update(ros::Time::now(), elapsed_time, false);
    //       }
    
    //       robot->write();
         
    //       ros_control_loop_rate->sleep();
    //     }
    // }

    void rosControlLoop() 
    {
        double data_control_loop_frequency;
        ros::param::get("~data_control_loop_frequency", data_control_loop_frequency);
        double dur = (double)(1/data_control_loop_frequency);
        
        ros::Duration d(dur);
        struct timespec tick;
        clock_gettime(CLOCK_REALTIME, &tick);
        //time for checking overrun
        struct timespec before;
        double overrun_time;
        
        ros::Time last_time = ros::Time::now();
        ros::Time current_time = ros::Time::now();
        ros::Duration elapsed_time;
        
        while(ros::ok()) {
            ros::Time this_moment(tick.tv_sec, tick.tv_nsec);
            
            robot->read(); 

            if (flag_reset_controllers) {
                current_time = ros::Time::now();
                elapsed_time = ros::Duration(current_time - last_time);
                last_time = current_time; 
                robot->setCommandToCurrentPosition();
                cm->update(ros::Time::now(), elapsed_time, true);
                flag_reset_controllers = false;
            }
            else {
                cm->update(this_moment, d);
            }

            robot->write();

            // ros_control_loop_rate->sleep();
            timespecInc(tick, d.nsec);
            // check overrun
            clock_gettime(CLOCK_REALTIME, &before);
            overrun_time = (before.tv_sec + double(before.tv_nsec)/1e+9) -  (tick.tv_sec + double(tick.tv_nsec)/1e+9);
            if(overrun_time > 0.0)
            {
                tick.tv_sec=before.tv_sec;
                tick.tv_nsec=before.tv_nsec;
            }

            clock_nanosleep(CLOCK_REALTIME, TIMER_ABSTIME, &tick, NULL);
        }
    }

    GaussDriver() 
    {
        reset_controller_subscriber = nh_.subscribe("/gauss/steppers_reset_controller", 10,  &GaussDriver::callbackTrajectoryGoal, this);
        debug_mode_subscriber = nh_.subscribe("/gauss/gauss_driver/debug_mode", 1,  &GaussDriver::callbackDebugMode, this);
        
        trajectory_result_subscriber = nh_.subscribe("/gauss_follow_joint_trajectory_controller/follow_joint_trajectory/result",
                10, &GaussDriver::callbackTrajectoryResult, this);

        ros::param::get("/gauss/hardware_version", hardware_version);
        ros::param::get("/gauss/can_protocol_version", can_protocol_version_);

        if (hardware_version != 1 && hardware_version != 2) {
            ROS_ERROR("Incorrect hardware version, should be 1 or 2");
            return;
        }

        if (can_protocol_version_ != 1 && can_protocol_version_ != 2) {
            ROS_ERROR("Incorrect protocol version, should be 1 or 2");
            return;
        }

        double ros_control_frequency;
        ros::param::get("~ros_control_loop_frequency", ros_control_frequency); 

        bool fake_communication;
        ros::param::get("~fake_communication", fake_communication);

        ROS_INFO("Starting gauss driver thread (frequency : %lf)", ros_control_frequency);

        if (fake_communication) {
            comm.reset(new FakeCommunication(hardware_version));
        }
        else {
            comm.reset(new GaussCommunication(hardware_version, can_protocol_version_));

        }
        
        int init_result = comm->init();
        if (init_result != 0) {
            return; // need to check last ROS_ERROR to get more info
        }
        
        ROS_INFO("Gauss communication has been successfully started");
       
        ros::Duration(0.1).sleep();
        flag_reset_controllers = true; 

        ROS_INFO("Start hardware control loop");
        comm->manageHardwareConnection();
        ros::Duration(0.5).sleep();

        ROS_INFO("Start hardware interface");
        robot.reset(new GaussHardwareInterface(comm.get()));
        
        ROS_INFO("Create controller manager");        
        cm.reset(new controller_manager::ControllerManager(robot.get(), nh_));
        ros::Duration(0.1).sleep();
        
        ROS_INFO("Starting ros control thread...");
        ros_control_loop_rate.reset(new ros::Rate(ros_control_frequency));
        ros_control_thread.reset(new std::thread(boost::bind(&GaussDriver::rosControlLoop, this)));

        ROS_INFO("Start Rpi Diagnostics...");
        rpi_diagnostics.reset(new RpiDiagnostics());

        ROS_INFO("Starting ROS interface...");
        bool learning_mode_activated_on_startup = true;
        ros_interface.reset(new RosInterface(comm.get(), rpi_diagnostics.get(), 
                    &flag_reset_controllers, learning_mode_activated_on_startup, hardware_version));

        // activate learning mode 
        comm->activateLearningMode(learning_mode_activated_on_startup);  

        debug_mode = false;      
    }

    /*
     * Problem : for joint_trajectory_controller, position command has no discontinuity
     * --> If the stepper motor missed some steps, we need to start at current position (given by the encoder)
     *  So current real position != current trajectory command, we need a discontinuity in controller command.
     *  We have to reset controllers to start from sensor position. 
     *  If we subscribe to trajectory /goal topic and reset when we receive a goal, it is often
     *  too late and trajectory will just be preempted.
     *
     *  So, in order to start from encoder position, we need to reset controller before we send the goal. If you
     *  send a new goal, be sure to send a message on /gauss_steppers_reset_controller BEFORE sending the goal.
     *
     *  This behavior is used in robot_commander node.
     *
     */
    void callbackTrajectoryGoal(const std_msgs::Empty& msg)
    {
        ROS_INFO("Received trajectory GOAL");
        robot->setCommandToCurrentPosition();  // set current command to encoder position
        cm->update(ros::Time::now(), ros::Duration(0.00), false); // reset controllers to allow a discontinuity in position command
        // cm->update(ros::Time::now(), ros::Duration(0.00), true); // reset controllers to allow a discontinuity in position command
        comm->synchronizeMotors(true);
    }

    void callbackDebugMode(const std_msgs::Bool& msg)
    {
        if(msg.data){
            debug_mode = true;
            g_debug_mode = debug_mode;
            ROS_INFO("debug_mode true");
        }else{
            debug_mode = false;
            g_debug_mode = debug_mode;
            ROS_INFO("debug_mode false");
        }
    }

    void callbackTrajectoryResult(const control_msgs::FollowJointTrajectoryActionResult& msg)
    {
        ROS_INFO("Received trajectory RESULT");
        comm->synchronizeMotors(false);
    }
};


int main(int argc, char **argv) 
{
    ros::init(argc, argv, "gauss_driver");
    ros::AsyncSpinner spinner(4);
    spinner.start();
    
    ros::NodeHandle nh;
    g_roslogger_pub = GaussLogger(nh);

    GaussDriver nd; 

    ros::waitForShutdown();
    
    ROS_INFO("shutdown node");
}

