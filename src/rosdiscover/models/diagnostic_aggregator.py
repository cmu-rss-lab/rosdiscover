# -*- coding: utf-8 -*-
from ..interpreter import model


@model('diagnostic_aggregator', 'aggregator_node')
def diagnostic_aggregator(c):
    base_path = c.read('~base_path', '')
    if '/' not in base_path:
        base_path = '/' + base_path

    c.read('~pub_rate', 1.0)

    c.read('~other_as_errors', False)

    # TODO create analyzer group
    # https://github.com/ros/diagnostics/blob/indigo-devel/diagnostic_aggregator/src/analyzer_group.cpp

    c.sub('/diagnostics', 'diagnostic_msgs/DiagnosticArray')
    c.pub('/diagnostics_agg', 'diagnostic_msgs/DiagnosticArray')
    c.pub('/diagnostics_toplevel_state', 'diagnostic_msgs/DiagnosticStatus')

    c.provide('/diagnostics_agg/add_diagnostics',
              'diagnostic_msgs/AddDiagnostics')
