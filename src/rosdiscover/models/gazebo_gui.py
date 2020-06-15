# -*- coding: utf-8 -*-
from ..interpreter import model


@model('gazebo_ros', 'gzclient')
def gazebo_gui(c):
    c.pub('/gazebo_gui/parameter_description', 'dynamic_recongfigure/ConfigDescription')
    c.pub('/gazebo_gui/parameter_updates', 'dynamic_reconfigure/Config')
    c.provide('/gazebo_gui/set_parameters', 'unknown type')  # FIXME
    c.sub('/clock', 'rosgraph_msgs/Clock')
