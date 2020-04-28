# -*- coding: utf-8 -*-
"""
Provides a simple command-line interface.
"""
from typing import Any, Mapping, Sequence
import argparse
import os

from loguru import logger
import roswire

from . import models
from .acme import AcmeGenerator
from .config import Config
from .interpreter import Interpreter, Model

CONFIG_HELP = f"""R|A YAML file defining the configuration.
- indicates stdin.
{Config.__doc__}"""

DESC = 'discovery of ROS architectures'


def _launch(config: Config) -> Interpreter:
    rsw = roswire.ROSWire()
    logger.info(f"reconstructing architecture for image [{config.image}]")
    # FIXME passing interpreter outside of the context is very weird/bad
    with Interpreter.for_image(config.image, config.sources) as interpreter:
        for fn_launch in config.launches:
            logger.info(f"simulating launch [{fn_launch}]")
            interpreter.launch(fn_launch)
        return interpreter


def _launch_config(args):
    config = Config.from_yaml_file(args.config)
    _launch(config)


def launch(args):
    """Simulates the architectural effects of a `roslaunch` command."""
    interpreter = _launch_config(args)
    output = [n.to_dict() for n in interpreter.nodes]
    if args.output is not None:
        with open(args.output,'w') as of:
            yaml.dump(output,of, default_flow_style=False)
    else:
        print(yaml.dump(output, default_flow_style=False))


def generate_acme(args):
    """Generates an Acme description for a given roslaunch command."""
    interpreter = _launch_config(args)
    nodes = [n.to_dict for n in interpreter.nodes]
    acme_gen = AcmeGenerator(nodes, args.acme)
    acme = acme_gen.generate_acme()
    if args.acme is not None:
        print("Writing Acme to %s" %args.acme)
        with open(args.acme,'w') as f:
            f.write(acme)
    else:
        print(acme)


def rostopic_list(args):
    # simulates the list command
    interpreter = _launch_config(args)
    topics = set()
    for node in interpreter.nodes:
        topics |= set(x for (x, _) in node.pubs | node.subs)
    print('\n'.join(sorted(topics)))


def rosservice_list(args):
    interpreter = _launch_config(args)
    services = set()
    for node in interpreter.nodes:
        services |= set(s for (s, _) in node.provides)
    print('\n'.join(sorted(services)))


def recover(args):
    node = args.node
    package = args.package
    print(f"Attempting to recover architecture for node [{node}] "
          f"in package [{package}]")

    config = Config.from_yaml_file(args.config)
    print(f"Using configuration: {config}")

    raise NotImplementedError


class MultiLineFormatter(argparse.HelpFormatter):
    def _split_lines(self, text, width):
        if text.startswith('R|'):
            return text[2:].splitlines()
        return argparse.HelpFormatter._split_lines(self, text, width)

def main():
    logger.enable('roswire')
    parser = argparse.ArgumentParser(description=DESC)
    subparsers = parser.add_subparsers()

    p = subparsers.add_parser(
        'launch',
        help='simulates the effects of a roslaunch.',
        formatter_class=MultiLineFormatter)
    p.add_argument('--output', type=str, help="file to output YAML to")
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.set_defaults(func=launch)

    p = subparsers.add_parser(
        'recover',
        help='attempts to recover an architectural model for a node.')
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.add_argument('package', type=str,
        help='The name of the package to which the node belongs.')
    p.add_argument('node', type=str, help='The name of the node.')
    p.set_defaults(func=recover)

    p = subparsers.add_parser(
        'rostopic',
        help='simulates the output of rostopic for a given configuration.',
        formatter_class=MultiLineFormatter)
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.set_defaults(func=rostopic_list)

    p = subparsers.add_parser(
        'rosservice',
        help='simulates the output of rosservice for a given configuration.',
        formatter_class=MultiLineFormatter)
    
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.set_defaults(func=rosservice_list)

    p = subparsers.add_parser('acme',
            help='generates Acme from a source file',
            formatter_class=MultiLineFormatter)
    p.add_argument("--acme", type=str, default="generated.acme", help='Output to the named Acme file')
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.set_defaults(func=generate_acme)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
