# -*- coding: utf-8 -*-
from ..interpreter import model


@model('twist_mux', 'twist_mux')
def twist_mux(c):
    topics = c.read("~topics", [])
    locks = c.read("~locks", [])
    c.pub('~cmd_vel', 'geometry_msgs/Twist')
    c.pub('diagnostics', 'diagnostic_msgs/DiagnosticArray')

    for topic in topics + locks:
        c.sub(f"/{topic['topic']}", "any")
