#!/usr/bin/env python

""" A couple of handy conversion utilities taken from the turtlebot_node.py script found in the
    turtlebot_node ROS package at:
    
    http://www.ros.org/wiki/turtlebot_node
    
"""

import PyKDL
import math
from geometry_msgs.msg import Point

def quat_to_angle(quat):
    rot = PyKDL.Rotation.Quaternion(quat.x, quat.y, quat.z, quat.w)
    return rot.GetRPY()[2]
        
def normalize_angle(angle):
    res = angle
    while res > pi:
        res -= 2.0 * pi
    while res < -pi:
        res += 2.0 * pi
    return res

def dot(v,w):
    x,y,z = v.x,v.y,v.z
    X,Y,Z = w.x,w.y,w.z
    return x*X + y*Y + z*Z

def length(v):
    x,y,z = v.x,v.y,v.z
    return math.sqrt(x*x + y*y + z*z)

def vector(b,e):
    x,y,z = b.x,b.y,b.z
    X,Y,Z = e.x,e.y,e.z
    return Point(X-x, Y-y, Z-z)

def unit(v):
    x,y,z = v.x,v.y,v.z
    mag = length(v)
    return Point(x/mag, y/mag, z/mag)

def distance(p0,p1):
    return length(vector(p0,p1))

def scale(v,sc):
    x,y,z = v.x,v.y,v.z
    return Point(x * sc, y * sc, z * sc)

def add(v,w):
    x,y,z = v.x,v.y,v.z
    X,Y,Z = w.x,w.y,w.z
    return Point(x+X, y+Y, z+Z)