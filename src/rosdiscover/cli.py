# -*- coding: utf-8 -*-
"""
Provides a simple command-line interface.
"""
import argparse
import os
import time
import typing as t

import pkg_resources
import yaml
from dockerblade.popen import Popen
from loguru import logger

from .acme import AcmeGenerator
from .config import Config
from .interpreter import Interpreter, SystemSummary
from .observer import Observer
from .recover import NodeRecoveryTool
from roswire.util import Stopwatch

DESC = 'discovery of ROS architectures'
CONFIG_HELP = """R|A YAML file defining the configuration.
- indicates stdin.
{Config.__doc__}"""


def recover(args: argparse.Namespace) -> None:
    """Provides static recovery of dynamic architecture models."""
    config = Config.from_yaml_string(args.config)
    if args.restrict_to and args.sources:
        for path in args.restrict_to:
            if not os.path.isabs(path):
                raise ValueError(f"Restricted path [{path}] should be absolute")
        if not args.entrypoint:
            raise ValueError("expected the name of an entrypoint to be specified")
        with NodeRecoveryTool.for_config(config) as tool:
            print(f"spun up the container: {tool}")
            model = tool.recover(
                package_name=args.package,
                node_name=args.node,
                entrypoint=args.entrypoint,
                sources=args.sources,
                path_restrictions=args.restrict_to
            )
    else:
        with NodeRecoveryTool.for_config(config) as tool:
            print(f"spun up container: {tool}")
            model = tool.recover_using_cmakelists(args.package, args.node)
    if args.save_to:
        print(f"saving recovered model to disk: {args.save_to}")
        model.save(args.save_to)
        print("saved recovered model to disk")


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
    to_ignore.append('\\unknown')

    acme_gen = AcmeGenerator(node_summaries, args.acme, args.jar, things_to_ignore=to_ignore)
    acme = acme_gen.generate_acme()

    acme_gen.generate_acme_file(acme)

    if args.acme is None:
        print(acme)

    if args.check:
        acme_gen.check_acme()


def _observe(obs: Observer, args: argparse.Namespace) -> SystemSummary:
    if args.do_launch:
        obs.launch_from_config(args.launch_sleep)
    summary = obs.observe()
    return summary


def _periodic_observe(obs: Observer,
                      interval: float,
                      args: argparse.Namespace) -> SystemSummary:

    launches: t.Sequence[Popen] = []
    if args.do_launch:
        launches = obs.launch_from_config(args.launch_sleep)
    try:
        process: t.Optional[Popen] = None
        if 'run_script' in args and args.run_script:
            logger.info(f"Running script {args.run_script} on container")
            process = obs.execute_script(args.run_script)
        summary = SystemSummary({})
        stopwatch = Stopwatch()
        iterations = 0
        go = True
        stopwatch.start()
        while go:
            try:
                iterations += 1
                obs_start = stopwatch.duration
                logger.info(f"Doing observation {iterations}")

                observation = obs.observe()
                summary = SystemSummary.merge(summary, observation)

                obs_end = stopwatch.duration
                logger.debug(f"Finished observation {iterations}; took {obs_end-obs_start:.3f} seconds.")

                if 'duration' in args:
                    go = stopwatch.duration < args.duration

                logger.debug(f'Sleeping for {interval:.3f} seconds')
                time.sleep(interval)

                if process:
                    result = process.poll()
                    if result:
                        if result != 0:
                            logger.error(f"{args.run_script} seems to have failed: {result}")
                            for line in process.stream:
                                logger.error(line)
                        process = None

                if 'duration' in args:
                    go = stopwatch.duration < args.duration
            except KeyboardInterrupt:
                go = False

        stopwatch.stop()

        if process and not process.poll():
            logger.warning(f"Observations seemed to have stopped before '{args.run_script}' completed.")

        logger.info(f"Finished observing - {iterations+1} observations in total after {stopwatch.duration:.3f} "
                    f"seconds.")
    finally:
        if process:
            process.kill()
        for launch in launches:
            logger.info(f"Killing {launch.args}")
            launch.kill()
    return summary


def do_observe(obs: Observer, args: argparse.Namespace) -> SystemSummary:
    if 'duration' in args or 'interval' in args:
        summary = _periodic_observe(obs, args.interval, args)
    else:
        summary = _observe(obs, args)
    return summary


def observe(args: argparse.Namespace) -> None:
    config = Config.from_yaml_string(args.config)
    if args.container:
        obs = Observer.for_container(args.container, config)
        summary = do_observe(obs, args)
    else:
        logger.warning("TODO: Make /startup-vnc.sh customizable")
        with Observer.for_image(config, start_script='/bin/bash /startup-vnc.sh') as obs:
            summary = do_observe(obs, args)
    output = summary.to_dict()
    if args.output:
        with open(args.output, 'w') as f:
            yaml.dump(output, f, default_flow_style=False)
    else:
        print(yaml.dump(output, default_flow_style=False))


def sources(args: argparse.Namespace) -> None:
    config = Config.from_yaml_string(args.config)
    config_with_sources = config.with_recovered_node_sources()

    with open(args.save_to, "w") as fh:
        yaml.dump(config_with_sources.to_dict(), fh, default_flow_style=False)


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


def main(args: t.Optional[t.Sequence[str]] = None) -> None:
    logger.enable('roswire')
    parser = argparse.ArgumentParser(description=DESC)
    subparsers = parser.add_subparsers()

    # ----------------- RECOVER --------------------
    p = subparsers.add_parser(
        'recover',
        help='statically recovers the dynamic architecture of a given node.',
        formatter_class=MultiLineFormatter,
    )
    p.add_argument(
        '--restrict-to',
        action='append',
        dest='restrict_to',
        help='the absolute container paths to which the static analysis should be restricted',
    )
    p.add_argument('--entrypoint', type=str, help='the function to use as an entry point')

    p.add_argument(
        '--save-to',
        help="the name of the file to which the recovered node model should be saved",
    )
    p.add_argument('config', type=argparse.FileType('r'), help=CONFIG_HELP)
    p.add_argument('package', type=str, help='the name of the package to which the node belongs')
    p.add_argument('node', type=str, help='the name of the node')
    p.add_argument(
        'sources',
        nargs='*',
        help='the paths of the translation unit source files for this node, relative to the package directory',
    )
    p.set_defaults(func=recover)

    # ----------------- SOURCES --------------------
    p = subparsers.add_parser(
        "sources",
        help="retrieves the sources for the nodes and nodelets in a given image",
    )
    p.add_argument("config", type=argparse.FileType("r"), help=CONFIG_HELP)
    p.add_argument(
        "--save-to",
        default="config-with-sources.yml",
        help="the name of the file to which the configuration with sources should be written",
    )
    p.set_defaults(func=sources)

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
    p.add_argument('--output', type=str, help='What file to output')
    p.add_argument('--duration', type=int, help='The amount of time (secs) to observe for')
    p.add_argument('--interval', type=float, help='The number of seconds to wait in between observations')
    p.add_argument('--run-script', type=str, help='A shell script to run on the container while observing')
    p.add_argument('--do-launch', action='store_true', help='Launch using the files specified in <config>')
    p.add_argument('--launch-sleep', type=float, help='Seconds to sleep between launch file execution', default=60.0)
    p.add_argument('--container', type=str, help='The container where the ROS system is running', default=None)
    p.add_argument('config', type=argparse.FileType('r'),
                   help='R|A YAML file defining the configuration (only the environment'
                   'information will be used).'
                   '- indicates stdin.'
                   '{Config.__doc__}')
    p.set_defaults(func=observe)

    parsed_args = parser.parse_args(args)
    if 'func' in parsed_args:
        parsed_args.func(parsed_args)
