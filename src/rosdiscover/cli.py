"""
Provides a simple command-line interface.
"""
import logging
import argparse

import yaml

from .workspace import Workspace
from .interpreter import Interpreter
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


def rostopic_list(args):
    # simulates the list command
    interpreter = _launch(args.filename, args.workspace)
    topics = set()
    for node in interpreter.nodes:
        topics |= set(x for (x, _) in node.pubs | node.subs)
    print('\n'.join(sorted(topics)))


def rosservice_list(args):
    interpreter = _launch(args.filename, args.workspace)
    services = set()
    for node in interpreter.nodes:
        services |= set(s for (s, _) in node.provides)
    print('\n'.join(sorted(services)))


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

    p = subparsers.add_parser(
        'rosservice',
        help='simulates the output of rosservice for a given configuration.')
    p.add_argument('filename', type=str, help='a ROS launch file')
    p.add_argument('--workspace', type=str, default='/ros_ws')
    p.set_defaults(func=rosservice_list)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
