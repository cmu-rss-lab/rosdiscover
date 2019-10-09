# -*- coding: utf-8 -*-
"""
Provides a simple command-line interface.
"""
import logging
import argparse

import yaml
import roswire

from .interpreter import Interpreter, Model
from .acme import AcmeGenerator
from . import models

DESC = 'discovery of ROS architectures'

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def _launch(name_image, fn_launch):
    rsw = roswire.ROSWire()
    logger.info("simulating launch [%s] for image [%s]",
                fn_launch, name_image)
    # FIXME passing interpreter outside of the context is very weird/bad
    with Interpreter.for_image(name_image) as interpreter:
        interpreter.launch(fn_launch)
        return interpreter


def launch(args):
    """Simulates the architectural effects of a `roslaunch` command."""
    interpreter = _launch(args.image, args.filename)
    output = [n.to_dict() for n in interpreter.nodes]
    print(yaml.dump(output, default_flow_style=False))


def generate_acme(args):
    """Generates an Acme description for a given roslaunch command."""
    interpreter = _launch(args.image, args.filename)
    nodes = [n.to_dict for n in interpreter.nodes]
    acme_gen = AcmeGenerator(nodes, args.filename)
    acme = acme_gen.generate_acme()
    if args.acme is not None:
        with open(args.acme,'w') as f:
            f.write(acme)
    else:
        print(acme)
    if (args.check):
        if args.acme is None:
            (results, _) = acme_gen.check_acme_string(acme)
        else:
            (results, _) = acme_gen.check_acme_file(args.acme)
        print(results)


def rostopic_list(args):
    # simulates the list command
    interpreter = _launch(args.image, args.filename)
    topics = set()
    for node in interpreter.nodes:
        topics |= set(x for (x, _) in node.pubs | node.subs)
    print('\n'.join(sorted(topics)))


def rosservice_list(args):
    interpreter = _launch(args.image, args.filename)
    services = set()
    for node in interpreter.nodes:
        services |= set(s for (s, _) in node.provides)
    print('\n'.join(sorted(services)))


def main():
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)
    logging.getLogger('roswire').addHandler(log_to_stdout)

    parser = argparse.ArgumentParser(description=DESC)
    subparsers = parser.add_subparsers()

    p = subparsers.add_parser(
        'launch',
        help='simulates the effects of a roslaunch.')
    p.add_argument('image', type=str,
                   help='name of a Docker image for a ROS application.')
    p.add_argument('filename', type=str,
                   help='path to a roslaunch file inside the Docker image.')
    p.set_defaults(func=launch)

    p = subparsers.add_parser(
        'rostopic',
        help='simulates the output of rostopic for a given configuration.')
    p.add_argument('image', type=str,
                   help='name of a Docker image for a ROS application.')
    p.add_argument('filename', type=str,
                   help='path to a roslaunch file inside the Docker image.')
    p.set_defaults(func=rostopic_list)

    p = subparsers.add_parser(
        'rosservice',
        help='simulates the output of rosservice for a given configuration.')
    p.add_argument('image', type=str,
                   help='name of a Docker image for a ROS application.')
    p.add_argument('filename', type=str,
                   help='path to a roslaunch file inside the Docker image.')
    p.set_defaults(func=rosservice_list)

    p = subparsers.add_parser('acme', help='generates Acme from a source file')
    p.add_argument('image', type=str,
                   help='name of a Docker image for a ROS application.')
    p.add_argument('filename', type=str,
                   help='path to a roslaunch file inside the Docker image.')
    p.add_argument("--acme", type=str, help='Output to the named Acme file')
    p.add_argument("--check", action='store_true', help='Typecheck the architecture and report any errors')
    p.set_defaults(func=generate_acme)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
