# -*- coding: utf-8 -*-
"""
Provides a simple command-line interface.
"""
from typing import Sequence
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


def _launch(name_image: str,
            launch_files: Sequence[str],
            sources: Sequence[str]
            ) -> Interpreter:
    rsw = roswire.ROSWire()
    logger.info("reconstructing architecture for image [%s]", name_image)
    # FIXME passing interpreter outside of the context is very weird/bad
    with Interpreter.for_image(name_image, sources) as interpreter:
        for fn_launch in launch_files:
            logger.info("simulating launch [%s]", fn_launch)
            interpreter.launch(fn_launch)
        return interpreter


def launch(args):
    """Simulates the architectural effects of a `roslaunch` command."""
    interpreter = _launch(args.image, args.filenames, args.sources)
    output = [n.to_dict() for n in interpreter.nodes]
    if args.output is not None:
        with open(args.output,'w') as of:
            yaml.dump(output,of, default_flow_style=False)
    else:
        print(yaml.dump(output, default_flow_style=False))


def generate_acme(args):
    """Generates an Acme description for a given roslaunch command."""
    interpreter = _launch(args.image, args.filenames, args.sources)
    nodes = [n.to_dict for n in interpreter.nodes]
    acme_gen = AcmeGenerator(nodes, args.filename)
    acme = acme_gen.generate_acme()
    if args.acme is not None:
        with open(args.acme,'w') as f:
            f.write(acme)
    else:
        print(acme)


def rostopic_list(args):
    # simulates the list command
    interpreter = _launch(args.image, args.filenames, args.sources)
    topics = set()
    for node in interpreter.nodes:
        topics |= set(x for (x, _) in node.pubs | node.subs)
    print('\n'.join(sorted(topics)))


def rosservice_list(args):
    interpreter = _launch(args.image, args.filenames, args.sources)
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
    p.add_argument('filenames', type=str, metavar='F', nargs='+',
                   help='paths to the roslaunch files inside the Docker image.')
    p.add_argument('--source', '-s', dest='sources', type=str, action='append',
                   help='sources that should be used to initialise the ROS workspace.')
    p.add_argument('--output', type=str, help="file to output YAML to")
    p.set_defaults(func=launch)

    p = subparsers.add_parser(
        'rostopic',
        help='simulates the output of rostopic for a given configuration.')
    p.add_argument('image', type=str,
                   help='name of a Docker image for a ROS application.')
    p.add_argument('filenames', type=str, metavar='F', nargs='+',
                   help='paths to the roslaunch files inside the Docker image.')
    p.add_argument('--source', '-s', dest='sources', type=str, action='append',
                   help='sources that should be used to initialise the ROS workspace.')
    p.set_defaults(func=rostopic_list)

    p = subparsers.add_parser(
        'rosservice',
        help='simulates the output of rosservice for a given configuration.')
    p.add_argument('image', type=str,
                   help='name of a Docker image for a ROS application.')
    p.add_argument('filenames', type=str, metavar='F', nargs='+',
                   help='paths to the roslaunch files inside the Docker image.')
    p.add_argument('--source', '-s', dest='sources', type=str, action='append',
                   help='sources that should be used to initialise the ROS workspace.')
    p.set_defaults(func=rosservice_list)

    p = subparsers.add_parser('acme', help='generates Acme from a source file')
    p.add_argument('image', type=str,
                   help='name of a Docker image for a ROS application.')
    p.add_argument('filenames', type=str, metavar='F', nargs='+',
                   help='paths to the roslaunch files inside the Docker image.')
    p.add_argument('--source', '-s', dest='sources', type=str, action='append',
                   help='sources that should be used to initialise the ROS workspace.')
    p.add_argument("--acme", type=str, help='Output to the named Acme file')
    p.set_defaults(func=generate_acme)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
