# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext


@model('*', 'error_reproducer')
def error_reproducer(c: NodeContext):
    """
    This model is intended for evaluation of rosdiscover on
    detecting misconfigurations. IT SHOULD NOT BE USED IN GENERAL.

    This model subscribes and publishes to topics that are passed
    in as parameters. In this way, we can easily construct a node
    that can be used to reproduce a configuration error that exists
    in our database.
    """
    topics = c.read("~topic", [])
    for t in topics:
        if t["direction"] == 'pub':
            c.pub(t["name"], t["type"])
        elif t["direction"] == 'sub':
            c.sub(t["name"], t["type"])
