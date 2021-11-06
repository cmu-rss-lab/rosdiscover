# -*- coding: utf-8 -*-
from ..interpreter import model, NodeContext


@model('*', 'error_reproducer')
def error_reproducer(c: NodeContext):
    topics = c.read("~topic", [])
    for t in topics:
        if t["direction"] == 'pub':
            c.pub(t["name"], t["type"])
        elif t["direction"] == 'sub':
            c.sub(t["name"], t["type"])
