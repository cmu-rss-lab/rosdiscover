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

from .interpreter import Interpreter, SystemSummary, NodeSummary

import roswire
import time
import json

DESC = 'discovery of ROS architectures'
CONFIG_HELP = """R|A YAML file defining the configuration.
- indicates stdin.
{Config.__doc__}"""


def _launch(config: Config) -> SystemSummary:
    logger.info(f"reconstructing architecture for image [{config.image}]")
    with Interpreter.for_image(config.image,
                               config.sources,
                               environment=config.environment
                               ) as interpreter:
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


def get_info(image, sources, environment, file, package, sleep_time):
    rsw = roswire.ROSWire()
    with rsw.launch(image, sources, environment=environment) as system:
        with system.roscore() as ros:
            ros.roslaunch(file,
                          package=package,
                          args={'gui': 'false'})

            time.sleep(sleep_time)
            node_names = list(ros.nodes)
            state = ros.state
            topic_to_type = ros.topic_to_type
            service_to_format = {}
            for service_name in state.services:
                service = ros.services[service_name]
                service_to_format[service_name] = service.format.name

    return node_names, state, topic_to_type, service_to_format


def create_dict(node_names, state, topic_to_type, service_to_format):
    node_summary_dict = {}
    for n in node_names:
        p = []
        for key in state.publishers:
            pubs = state.publishers[key]
            for i in pubs:
                if i == n:
                    p.append((topic_to_type[key], key))
        s = []
        for key in state.subscribers:
            subs = state.subscribers[key]
            for i in subs:
                if i == n:
                    s.append((topic_to_type[key], key))
        serv = []
        for key in service_to_format:
            path = n + '/'
            if path in key:
                serv.append((service_to_format[key], key))
        obj = NodeSummary('', n, '/', '', '', False, '', False, p, s, [], [], [], serv, [], [])
        node_summary_dict.update({n: obj})
    return node_summary_dict


def dynamic_analysis(args):
    with open(args.config, 'r') as f:
        data = yaml.safe_load(f)
    if args.output:
        f = open(args.output, "w")
    else:
        f = open("arch.yml", "w")
    logger.enable('roswire')
    image = data['image']
    sources = data['sources']
    if 'environment' in data:
        environment = data['environment']
    if args.sleep:
        sleep_time = args.sleep
    else:
        sleep_time = 30
    node_names, state, topic_to_type, service_to_format = get_info(image, sources, environment,
                                                                   args.launchfile, args.package, sleep_time)
    node_summary_dict = create_dict(node_names, state, topic_to_type, service_to_format)
    f.write(json.dumps(node_summary_dict, indent=4, separators=(". ", " = ")))
    f.close()


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
        'dynamic',
        help='Generates a dynamic analysis using rosnode list',
        formatter_class=MultiLineFormatter)

    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.add_argument('--output', type=str)
    p.add_argument('--sleep', type=int)
    p.add_argument('--package', type=str, help="package for roslaunch")
    p.add_argument('--launchfile', type=str, help="launchfile for roslaunch")
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)

    p.set_defaults(func=dynamic_analysis)

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
