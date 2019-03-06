#!/usr/bin/env python

class Position:
    class RPY: 
       
        def __init__(self, roll = 0, pitch = 0, yaw = 0): 
            self.roll = roll
            self.pitch = pitch 
            self.yaw = yaw
    class Point: 

        def __init__(self, x = 0, y = 0, z = 0 ): 
            self.x = x
            self.y = y
            self.z = z

    class Quaternion: 

        def __init__(self, x = 0, y = 0, z = 0, w = 0): 
            self.x = x
            self.y = y 
            self.z = z
            self.w = w

    def __init__(self, name = "", joints = [0,0,0,0,0,0], rpy = RPY(), point = Point(), quaternion = Quaternion()): 

        self.name = name 
        self.joints = joints 
        self.rpy = rpy
        self.point = point
        self.quaternion = quaternion

       
