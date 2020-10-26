# -*- coding: utf-8 -*-
"""
Provides a simple command-line interface.
"""
import argparse

from loguru import logger
import yaml
import pkg_resources

from .acme import AcmeGenerator
from .config import Config
from .interpreter import Interpreter, SystemSummary

DESC = 'discovery of ROS architectures'
CONFIG_HELP = """R|A YAML file defining the configuration.
- indicates stdin.
{Config.__doc__}"""


def _launch(config: Config) -> SystemSummary:
    logger.info(f"reconstructing architecture for image [{config.image}]")
    with Interpreter.for_config(config) as interpreter:
        ros_dist = interpreter.app.description.distribution
        logger.info(f'Detected ROS {ros_dist.ros} version: {ros_dist.name}')
        for fn_launch in config.launches:
            logger.info(f"simulating launch [{fn_launch}]")
            interpreter.launch(fn_launch)
        return interpreter.summarise()


def _launch_config(args) -> SystemSummary:
    config = Config.from_yaml_string(args.config)
    return _launch(config)


def launch(args) -> None:
    """Simulates the architectural effects of a `roslaunch` command."""
    summary = _launch_config(args)
    output = summary.to_dict()
    if args.output:
        with open(args.output, 'w') as f:
            yaml.dump(output, f, default_flow_style=False)
    else:
        print(yaml.dump(output, default_flow_style=False))


def generate_acme(args):
    """Generates an Acme description for a given roslaunch command."""
    summary = _launch_config(args)
    node_summaries = summary.values()

    acme_gen = AcmeGenerator(node_summaries, args.acme, args.jar)
    acme = acme_gen.generate_acme()

    acme_gen.generate_acme_file(acme)

    if args.acme is None:
        print(acme)
        if args.check:
            acme_gen.check_acme(acme)
    else:
        if args.check:
            acme_gen.check_acme()


def rostopic_list(args) -> None:
    summary = _launch_config(args)
    topics = set()
    for node in summary.values():
        topics |= set(t.name for t in set(node.pubs) | set(node.subs))
    print('\n'.join(sorted(topics)))


def rosservice_list(args) -> None:
    summary = _launch_config(args)
    services = set()
    for node in summary.values():
        services |= set(s.name for s in node.provides)
    print('\n'.join(sorted(services)))


class MultiLineFormatter(argparse.HelpFormatter):
    def _split_lines(self, text, width):
        if text.startswith('R|'):
            return text[2:].splitlines()
        return argparse.HelpFormatter._split_lines(self, text, width)


def main() -> None:
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

    acme_jar_path = pkg_resources.resource_filename(__name__, 'acme/lib/acme.standalone-ros.jar')
    p = subparsers.add_parser('acme',
                              help='generates Acme from a source file',
                              formatter_class=MultiLineFormatter)
    p.add_argument("--acme", type=str, default="generated.acme", help='Output to the named Acme file')

    p.add_argument("--check", "-c", action='store_true')
    p.add_argument("--jar", type=str, help='Pointer to the Acme jar file', default=acme_jar_path)

    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.set_defaults(func=generate_acme)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
