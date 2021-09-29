# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext


@model("teleop_twist_joy", "teleop_node")
def teleop_node(c: NodeContext) -> None:
    c.sub("joy", "sensor_msgs/Joy")
    c.pub("cmd_vel", "geometry_msgs/Twist")

    c.read("~enable_button", 0)
    c.read("~enable_turbo_button", -1)
    c.read("~axis_linear", 1)
    c.read("~scale_linear", 0.5)
    c.read("~scale_linear_turbo", 1.0)
    c.read("~axis_angular", 0)
    c.read("~scale_angular", 1.0)
    c.read("~scale_angular_turbo", c.read("~scale_angular"))
