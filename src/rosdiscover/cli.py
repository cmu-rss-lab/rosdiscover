"""
Provides a simple command-line interface.
"""
import logging
import argparse

import yaml

from .workspace import Workspace
from .interpreter import Interpreter
from .acme import AcmeGenerator
from . import models

DESC = 'discovery of ROS architectures'

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def _launch(fn_launch, dir_workspace):
    logger.info("simulating launch [%s] inside workspace [%s]",
                fn_launch, dir_workspace)

    workspace = Workspace(dir_workspace)
    interpreter = Interpreter(workspace)
    interpreter.launch(fn_launch)
    return interpreter


def launch(args):
    """
    Simulates the architectural effects of a `roslaunch` command.
    """
    interpreter = _launch(args.filename, args.workspace)
    output = [n.to_dict() for n in interpreter.nodes]
    print(yaml.dump(output, default_flow_style=False))

def generate_acme(args):
    """
    Generates an Acme description from the launch file
    """
    interpreter = _launch(args.filename, args.workspace)
    nodes = [n.to_dict for n in interpreter.nodes]
    acme_gen = AcmeGenerator(nodes,args.filename)
    print(acme_gen.generate_acme())


def rostopic_list(args):
    # simulates the list command
    interpreter = _launch(args.filename, args.workspace)
    topics = set()
    for node in interpreter.nodes:
        topics |= set(x for (x, _) in node.pubs | node.subs)
    print('\n'.join(sorted(topics)))


def main():
    # log_to_stdout = logging.StreamHandler()
    # log_to_stdout.setLevel(logging.DEBUG)
    # logging.getLogger('rosdiscover').addHandler(log_to_stdout)

    parser = argparse.ArgumentParser(description=DESC)
    subparsers = parser.add_subparsers()

    p = subparsers.add_parser(
        'launch',
        help='simulates the effects of a roslaunch.')
    p.add_argument('filename', type=str, help='a ROS launch file')
    p.add_argument('--workspace', type=str, default='/ros_ws')
    p.set_defaults(func=launch)

    p = subparsers.add_parser(
        'rostopic',
        help='simulates the output of rostopic for a given configuration.')
    p.add_argument('filename', type=str, help='a ROS launch file')
    p.add_argument('--workspace', type=str, default='/ros_ws')
    p.set_defaults(func=rostopic_list)

    p = subparsers.add_parser('acme', help='generates Acme from a source file')
    p.add_argument('filename', type=str, help='a ROS launch file')
    p.add_argument('--workspace', type=str, default='/ros_ws')
    p.set_defaults(func=generate_acme)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
