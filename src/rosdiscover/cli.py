# -*- coding: utf-8 -*-
"""
Provides a simple command-line interface.
"""
import argparse
import os

from loguru import logger
import pkg_resources
import yaml

from .acme import AcmeGenerator
from .config import Config
from .interpreter import Interpreter, SystemSummary
from .observer import Observer
from .recover import NodeRecoveryTool

DESC = 'discovery of ROS architectures'
CONFIG_HELP = """R|A YAML file defining the configuration.
- indicates stdin.
{Config.__doc__}"""


def recover(args: argparse.Namespace) -> None:
    """Provides static recovery of dynamic architecture models."""
    config = Config.from_yaml_string(args.config)
    for path in args.restricted_to:
        if not os.path.isabs(path):
            raise ValueError(f"Restricted path [{path}] should be absolute")
    with NodeRecoveryTool.for_config(config) as tool:
        print(f"spun up the container: {tool}")
        tool.recover(args.package, args.node, args.entry, args.sources, args.restrict_to)


def _launch(config: Config) -> SystemSummary:
    logger.info(f"reconstructing architecture for image [{config.image}]")
    with Interpreter.for_config(config) as interpreter:
        ros_dist = interpreter.app.description.distribution
        logger.info(f'Detected {ros_dist.ros} version: {ros_dist.name}')
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

    # Warn of any undefined nodes
    for node_summary in summary.unresolved:
        print(f'Warning: {node_summary.name} in package: {node_summary.package} could not be found.')
    if args.output:
        with open(args.output, 'w') as f:
            yaml.dump(output, f, default_flow_style=False)
    else:
        print(yaml.dump(output, default_flow_style=False))


def generate_acme(args) -> None:
    """Generates an Acme description for a given roslaunch command."""
    summary: SystemSummary
    if args.from_yml:
        arr = yaml.load(args.from_yml, Loader=yaml.SafeLoader)
        assert isinstance(arr, list)
        summary = SystemSummary.from_dict(arr)
    else:
        summary = _launch_config(args)
    node_summaries = summary.values()

    to_ignore = []
    if args.ignore_list:
        to_ignore = [line.rstrip() for line in args.ignore_list]

    acme_gen = AcmeGenerator(node_summaries, args.acme, args.jar, things_to_ignore=to_ignore)
    acme = acme_gen.generate_acme()

    acme_gen.generate_acme_file(acme)

    if args.acme is None:
        print(acme)

    if args.check:
        acme_gen.check_acme()


def _observe(args) -> SystemSummary:
    config = Config.from_yaml_string(args.config)
    obs = Observer.for_container(args.container, config)
    summary = obs.observe()
    return summary


def observe(args) -> None:
    summary = _observe(args)
    output = summary.to_dict()
    if args.output:
        with open(args.output, 'w') as f:
            yaml.dump(output, f, default_flow_style=False)
    else:
        print(yaml.dump(output, default_flow_style=False))


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

    # ----------------- RECOVER --------------------
    p = subparsers.add_parser(
        'recover',
        help='statically recovers the dynamic architecture of a given node.',
        formatter_class=MultiLineFormatter,
    )
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.add_argument('package', type=str, help='the name of the package to which the node belongs')
    p.add_argument('node', type=str, help='the name of the node')
    p.add_argument('entry', type=str, help='the function to use as an entry point')
    p.add_argument(
        'sources',
        nargs='+',
        help='the paths of the translation unit source files for this node, relative to the package directory',
    )
    p.add_argument(
        'restrict-to',
        nargs='+',
        type=str,
        default=[],
        help='the absoulate container paths to restrict static analysis to'
    )
    p.set_defaults(func=recover)

    # ----------------- LAUNCH --------------------
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

    # ----------------- ROSSERVICE --------------------
    p = subparsers.add_parser(
        'rosservice',
        help='simulates the output of rosservice for a given configuration.',
        formatter_class=MultiLineFormatter)
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.set_defaults(func=rosservice_list)

    # ----------------- ACME --------------------
    acme_jar_path = pkg_resources.resource_filename(__name__, 'acme/lib/acme.standalone-ros.jar')
    p = subparsers.add_parser('acme',
                              help='generates Acme from a source file',
                              formatter_class=MultiLineFormatter)
    p.add_argument("--acme",
                   type=str,
                   default="generated.acme",
                   help='Output to the named Acme file')
    p.add_argument("--from-yml",
                   type=argparse.FileType('r'),
                   help=("A YML file (in the format produced by the 'launch' default command) "
                         "from which to derive the architecture"))
    p.add_argument('--ignore-list',
                   type=argparse.FileType('r'),
                   help="A file containing a list of topics, services, actions to ignore.")
    p.add_argument("--check", "-c", action='store_true')
    p.add_argument("--jar", type=str, help='Pointer to the Acme jar file', default=acme_jar_path)

    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.set_defaults(func=generate_acme)

    # ----------------- OBSERVE --------------------
    p = subparsers.add_parser('observe',
                              help='observes a robot running in a container and produces an '
                                   'architecture',
                              formatter_class=MultiLineFormatter)
    p.add_argument('--acme', action='store_true', help='Generate an Acme file instead of the YAML')
    p.add_argument('--output', type=str, help='What file to output')
    p.add_argument('container', type=str, help='The container where the ROS system is running')
    p.add_argument('config', type=argparse.FileType('r'),
                   help='R|A YAML file defining the configuration (only the environment'
                   'information will be used).'
                   '- indicates stdin.'
                   '{Config.__doc__}')
    p.set_defaults(func=observe)

    args = parser.parse_args()
    if 'func' in args:
        args.func(args)
