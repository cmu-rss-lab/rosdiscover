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
