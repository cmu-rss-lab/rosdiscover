# -*- coding: utf-8 -*-
import argparse
import shlex

from ..interpreter import model, NodeContext


# Models a call to rostopic from <node/> in launch
@model('rostopic', 'rostopic')
def rostopic(c: NodeContext) -> None:
    parser = argparse.ArgumentParser()

    def publsh(args: argparse.Namespace):
        c.pub(args.topic)

    subparsers = parser.add_subparsers()
    p = subparsers.add_parser('pub', help='Publishes a message')
    p.add_argument('-l', action='store_true')
    p.add_argument('topic', type=str)
    p.add_argument('msg', type=str)

    parser.parse_args(shlex.split(c.args))
