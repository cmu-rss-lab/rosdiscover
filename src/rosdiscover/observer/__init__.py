# -*- coding: utf-8 -*-
"""
This module is used to model the architecture of ROS systems running in a container.

The main class within this module is :class:`Observer`, which acts as a mediator for
executing commands to extract the architecture
"""
from .observer import ExecutionError, Observer
from .ros1 import ROS1Observer
from .ros2 import ROS2Observer
