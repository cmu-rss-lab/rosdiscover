"""
Provides a simple command-line interface.
"""
import logging
import argparse

import yaml

from .workspace import Workspace
from .vm import VM
from . import models

DESC = 'discovery of ROS architectures'

logger = logging.getLogger(__name__)  # type: logging.Logger
logger.setLevel(logging.DEBUG)


def launch(fn_launch, dir_workspace):
    # type: (str, str) -> None:
    """
    Simulates the architectural effects of a `roslaunch` command.
    """
    logger.info("simulating launch [%s] inside workspace [%s]",
                fn_launch, dir_workspace)

    workspace = Workspace(dir_workspace)
    vm = VM(workspace)
    vm.launch(fn_launch)

    output = [n.to_dict() for n in vm.nodes]
    print(yaml.dump(output, default_flow_style=False))


def main():
    log_to_stdout = logging.StreamHandler()
    log_to_stdout.setLevel(logging.DEBUG)
    logging.getLogger('rosdiscover').addHandler(log_to_stdout)

    # simulates the architectural effects of a ROS launch
    parser = argparse.ArgumentParser(description=DESC)
    parser.add_argument('filename', type=str, help='a ROS launch file')
    parser.add_argument('--workspace', type=str, default='/ros_ws')
    args = parser.parse_args()
    launch(args.filename, args.workspace)
