#!/usr/bin/env python

class CommandStatus:
    
    # Success
    SUCCESS = 1
    STOPPED = 2
    WAITING = 6

    # Rejected
    GOAL_STILL_ACTIVE  = 10
    INVALID_PARAMETERS = 11
    LEARNING_MODE_ON   = 12
    JOYSTICK_ENABLED   = 13
    HARDWARE_NOT_OK    = 14

    # Aborted
    NO_PLAN_AVAILABLE   = 20
    PLAN_FAILED         = 21
    CONTROLLER_PROBLEMS = 22
    STILL_RUNNING       = 23
    TOOL_FAILED         = 24
    SEQUENCE_FAILED     = 25

    # ROS error
    ROS_ERROR           = 30

