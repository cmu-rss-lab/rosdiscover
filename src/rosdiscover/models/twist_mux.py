# -*- coding: utf-8 -*-
from ..interpreter import model


@model('twist_mux', 'twist_mux')
def twist_mux(c):
    c.pub('cmd_vel_out', 'geometry_msgs/Twist')
    c.pub('diagnostics', 'diagnostic_msgs/DiagnosticArray')
    c.sub('/clock', 'rosgraph_msgs/Clock')
    c.sub('cmd_vel', 'unknown type')
    c.sub('e_stop', 'unknown type')
    c.sub('/move_base/cmd_vel', 'unknown type')
    c.sub('/pad_teleop/cmd_vel', 'unknown type')
