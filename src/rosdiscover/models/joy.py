# -*- coding: utf-8 -*-
# from ..interpreter import model

# Removed from handwritten because we can recover this node statically
# @model('joy', 'joy_node')
def joy(c):
    c.pub('diagnostics', 'diagnostic_msgs/DiagnosticArray')
    c.pub('joy', 'sensor_msgs/Joy')
    c.sub('/clock', 'rosgraph_msgs/Clock')
    c.sub('set_feedback', 'unknown type')  # FIXME
