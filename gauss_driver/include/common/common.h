#ifndef __COMMON_H__
#define __COMMON_H__
#include "std_msgs/String.h"
#include <ros/ros.h>
extern bool g_debug_mode;

class GaussLogger {
    public:
    ros::Publisher log_pub;
    GaussLogger(){};
    GaussLogger(ros::NodeHandle &nh){
        log_pub = nh.advertise<std_msgs::String>("/gauss/gauss_node/gauss_driver_log", 1);
    }

    GaussLogger& operator=(const GaussLogger& log) {
        if(this == &log)
            return *this;
        
        log_pub = log.log_pub;
        return *this; 
    }

    void publish(std_msgs::String &log_msg){
        if(g_debug_mode) {
            log_pub.publish(log_msg);
        }
    }
};

#endif