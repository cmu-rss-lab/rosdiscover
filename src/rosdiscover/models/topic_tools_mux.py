# -*- coding: utf-8 -*-
from ..interpreter import model

import argparse


@model("topic_tools", "mux")
def mux(c):
    c.read("~lazy", False)
    c.read("~initial_topic", "")

    c.provide("mux/select", "topic_tools/MuxSelect")
    c.provide("mux/add", "topic_tools/MuxAdd")
    c.provide("mux/delete", "topic_tools/MuxDelete")

    parser = argparse.ArgumentParser("topic_tools/mux")
    parser.add_argument("outopic", type=str)
    parser.add_argument("intopic", nargs="+", type=str)
    topics = parser.parse_args(c.args.split())

    c.pub(topics.outopic, "any")
    for in_topic in topics.intopic:
        c.sub(in_topic, "any")

@model("topic_tools", "relay")
def relay(c):
    c.read("~unreliable", False)
    c.read('~lazy', False)
    c.read('~stealth', False)
    # TODO: Chris: how should we treat this? I think this means a topic is created
    # that by default is the same is intopic, but could be set by a parameter
    # http://wiki.ros.org/topic_tools/relay
    c.read('~monitor_topic', 'intopic')
    c.read('~monitor_rate', 1.0)

    parser = argparse.ArgumentParser("topic_tools/relay")
    parser.add_argument("intopic", type=str)
    parser.add_argument("outtopic", type=str)
    topics = parser.parse_args(c.args.split())
    c.pub(topics.outtopic, 'any')
    c.sub(topics.intopic, 'any')