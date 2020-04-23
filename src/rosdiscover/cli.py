# -*- coding: utf-8 -*-
"""
Provides a simple command-line interface.
"""
from typing import Any, Mapping, Sequence
import logging
import argparse
import os.path
from os import path

import roswire
import yaml

from .interpreter import Interpreter, Model
from .acme import AcmeGenerator
from . import models

DESC = 'discovery of ROS architectures'

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def _read_configuration(args) -> Mapping[str, Any]:
    config = {}
    if args.config is not None:
        config = yaml.load(args.config, Loader=yaml.SafeLoader)
    return config


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

def _launch_config(args):
    config = _read_configuration(args)
    if 'image' not in config.keys():
        raise Exception("'image' is undefined in configuration")
    if 'launches' not in config.keys() or not isinstance(config['launches'], list):
        raise Exception("'launches' is missing or is not a list in configuration")
 
    if 'sources' not in config.keys():
        config["sources"] = []
    
    if not isinstance(config['sources'], list):
        raise Exception("'sources' is not a list in the configuration")
    _launch(config["image"], config["launches"], config["sources"])

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

class MultiLineFormatter(argparse.HelpFormatter):
    def _split_lines(self, text, width):
        if text.startswith('R|'):
            return text[2:].splitlines()
        return argparse.HelpFormatter._split_lines(self, text, width)

config_help="""R|The YAML file to read configuration parameters from. 
- indicates stdin
  Should have the following parts:
    image:(required) name of a Docker image for a ROS application.
    launches:(required)
      - an aray of paths to the roslaunch files inside the Docker image.
    sources:(optional)
     - an array of sources that should be used to initialise the ROS workspace."""

def main():

    

    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)
    logging.getLogger('roswire').addHandler(log_to_stdout)

    parser = argparse.ArgumentParser(description=DESC)
    subparsers = parser.add_subparsers()

    p = subparsers.add_parser(
        'launch',
        help='simulates the effects of a roslaunch.', formatter_class=MultiLineFormatter)
    p.add_argument('--output', type=str, help="file to output YAML to")
    p.add_argument('config', type=argparse.FileType('r'), help=config_help)

    p.set_defaults(func=launch)

    p = subparsers.add_parser(
        'rostopic',
        help='simulates the output of rostopic for a given configuration.', formatter_class=MultiLineFormatter)
    
    p.add_argument('config', type=argparse.FileType('r'), help=config_help)

    p.set_defaults(func=rostopic_list)

    p = subparsers.add_parser(
        'rosservice',
        help='simulates the output of rosservice for a given configuration.', formatter_class=MultiLineFormatter)
    
    p.add_argument('config', type=argparse.FileType('r'), help=config_help)
    p.set_defaults(func=rosservice_list)

    p = subparsers.add_parser('acme', help='generates Acme from a source file', formatter_class=MultiLineFormatter)
    p.add_argument("--acme", type=str, default="generated.acme", help='Output to the named Acme file')
    p.add_argument('config', type=argparse.FileType('r'), help=config_help)
    p.set_defaults(func=generate_acme)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
