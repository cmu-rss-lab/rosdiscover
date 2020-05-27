# -*- coding: utf-8 -*-
from ..interpreter import model


@model('pdw_turtlebot3_teleop', 'pdw_turtlebot3_teleop_key')
def pdw_turtlebot3_teleop_key(c):
    c.pub('cmd_vel', 'geometry_msgs/Twist')
    c.read('model', 'burger')
