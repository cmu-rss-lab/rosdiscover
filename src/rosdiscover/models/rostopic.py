# -*- coding: utf-8 -*-
import argparse
import shlex

from ..interpreter import model, NodeContext


# Models a call to rostopic from <node/> in launch
@model('rostopic', 'rostopic')
def rostopic(c: NodeContext) -> None:
    parser = argparse.ArgumentParser()

    def publish(args: argparse.Namespace):
        c.pub(args.topic, args.type)

    def subscribe(args: argparse.Namespace):
        c.pub(args.topic, 'any')

    subparsers = parser.add_subparsers()
    p = subparsers.add_parser('pub', help='Publishes a message')
    p.add_argument('-l', '--latch', action='store_true')
    p.add_argument('-r', metavar='rate', type=int)
    p.add_argument('-1', '--once', action='store_true')
    p.add_argument('topic', type=str)
    p.add_argument('type', type=str)
    p.add_argument('msg', type=str)
    p.set_defaults(func=publish)

    p = subparsers.add_parser('echo', help='Subscribes to a topic')
    p.add_argument('--offset', action='store_true')
    p.add_argument('--filter', type=str)
    p.add_argument('-c', metavar='clear')
    p.add_argument('-b', type=str, metavar='bag')
    p.add_argument('-p', action='store_true', metavar='print_friendly')
    p.add_argument('-w', type=int, metavar='width_fixed')
    p.add_argument('--nostr', action='store_true')
    p.add_argument('--noarr', action='store_true')
    p.add_argument('-n', type=int, metavar='count')
    p.add_argument("topic", type=str)
    p.set_defaults(func=subscribe)

    args = parser.parse_args(shlex.split(c.args))
    if 'func' in args:
        args.func(args)
