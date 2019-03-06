#!/usr/bin/env python

class RobotCommanderException(Exception):
    def __init__(self, status, message):
        self.status = status
        self.message = message
