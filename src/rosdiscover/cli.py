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
from .interpreter import Interpreter, NodeSummary

import roswire
import time

DESC = 'discovery of ROS architectures'
CONFIG_HELP = """R|A YAML file defining the configuration.
- indicates stdin.
{Config.__doc__}"""


def _launch(config: Config) -> Interpreter:
    logger.info(f"reconstructing architecture for image [{config.image}]")
    # FIXME passing interpreter outside of the context is very weird/bad
    with Interpreter.for_image(config.image,
                               config.sources,
                               environment=config.environment
                               ) as interpreter:
        for fn_launch in config.launches:
            logger.info(f"simulating launch [{fn_launch}]")
            interpreter.launch(fn_launch)
        return interpreter


def _launch_config(args) -> Interpreter:
    config = Config.from_yaml_file(args.config)
    return _launch(config)


def launch(args) -> None:
    """Simulates the architectural effects of a `roslaunch` command."""
    interpreter = _launch_config(args)
    output = [n.to_dict() for n in interpreter.nodes]
    if args.output:
        with open(args.output, 'w') as f:
            yaml.dump(output, f, default_flow_style=False)
    else:
        print(yaml.dump(output, default_flow_style=False))


def generate_acme(args):
    """Generates an Acme description for a given roslaunch command."""
    interpreter = _launch_config(args)
    # nodes = [n.to_dict for n in interpreter.nodes]

    acme_gen = AcmeGenerator(interpreter.nodes, args.acme, args.jar)
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
    # simulates the list command
    interpreter = _launch_config(args)
    topics = set()
    for node in interpreter.nodes:
        topics |= set(x for (x, _) in set(node.pubs) | set(node.subs))
    print('\n'.join(sorted(topics)))


def rosservice_list(args) -> None:
    interpreter = _launch_config(args)
    services = set()
    for node in interpreter.nodes:
        services |= set(s for (s, _) in node.provides)
    print('\n'.join(sorted(services)))


def toString(line):
    s = ''
    l1 = list(line)
    if len(l1) == 0:
        return '[]'
    for i in l1:
        (fmt, topic) = i
        s = s + "\n  - format: " + fmt + "\n    name: " + topic + "\n"
    return s


def get_info(image, sources, environment):
    rsw = roswire.ROSWire()
    with rsw.launch(image, sources, environment=environment) as system:
        with system.roscore() as ros:
            ros.roslaunch('turtlebot3_house.launch',
                          package='turtlebot3_gazebo',
                          args={'gui': 'false'})

            time.sleep(30)
            node_names = list(ros.nodes)
            state = ros.state
            topic_to_type = ros.topic_to_type
            service_to_format = {}
            for service_name in state.services:
                service = ros.services[service_name]
                service_to_format[service_name] = service.format.name

    return node_names, state, topic_to_type, service_to_format


def create_dict(node_names, state, topic_to_type, service_to_format):
    nodeSummaryDict = {}
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
        nodeSummaryDict.update({n: obj})
    return nodeSummaryDict


def dynamic_analysis(args):
    print(args)
    with open(args.config.name, 'r') as r:
        data = yaml.safe_load(r)
    r.close()
    f = open("Arch.yml", "w")
    logger.enable('roswire')
    image = data['image']
    sources = data['sources']
    if 'environment' in data:
        environment = data['environment']
    node_names, state, topic_to_type, service_to_format = get_info(image, sources, environment)
    nodeSummaryDict = create_dict(node_names, state, topic_to_type, service_to_format)
    for i in nodeSummaryDict:
        obj = nodeSummaryDict[i]
        f.write(f"- action-clients: {toString(obj.action_clients)}\n  ")
        f.write(f"action-servers: {toString(obj.action_servers)}\n  ")
        f.write(f"filename: {obj.filename}\n  fullname: {obj.fullname}\n ")
        f.write(f"kind: {obj.kind}\n  name: {obj.name}\n")
        f.write(f"namespace: {obj.namespace}\n  nodelet: {obj.nodelet}\n ")
        f.write(f"package: {obj.package}\n  placeholder: {obj.placeholder}\n  ")
        f.write(f"provides: {toString(obj.provides)}\n  pubs: {toString(obj.pubs)}\n  ")
        f.write(f"reads: {toString(obj.reads)}\n  subs: {toString(obj.subs)}\n  ")
        f.write(f"uses: {toString(obj.uses)}\n  writes: {toString(obj.writes)}\n\n")
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
