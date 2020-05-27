# -*- coding: utf-8 -*-
from ..interpreter import model


@model('turtlebot3_teleop', 'turtlebot3_teleop_key')
def turtlebot3_teleop_key(c):
    c.pub('cmd_vel', 'geometry_msgs/Twist')
    c.read('model', 'burger')
