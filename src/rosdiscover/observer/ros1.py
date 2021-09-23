# -*- coding: utf-8 -*-
__all__ = ('ROS1Observer',)

import os
import tempfile
import time
import typing as t

from dockerblade.popen import Popen
from loguru import logger
from roswire.common import SystemState

from .nodeinfo import NodeInfo
from .observer import ExecutionError, Observer
from ..interpreter import NodeContext, SystemSummary
from ..launch import Launch

_NODES_TO_FILTER_OUT = ('/rosout',)
_TOPICS_TO_FILTER_OUT = ('/rosout', '/rosout_agg')
_SERVICES_TO_FILTER_OUT = ('set_logger_level', 'get_loggers')


class ROS1Observer(Observer):

    def observe(self) -> SystemSummary:
        """Observe the state of the running system and produce a summary of the architecture."""
        nodecontexts = []
        with self._app_instance.ros1() as ros:
            nodes = self._transform_state_to_nodeinfo(ros.state)
            for node in nodes:
                nodecontext = node.make_node_context(ros, self._app_instance)
                nodecontexts.append(nodecontext)

        return self._summarise(nodecontexts)

    def _summarise(self, contexts: t.Collection[NodeContext]) -> SystemSummary:
        """
        Produce an immutable description of the system architecture from the collection of
        NodeContexts.

        Parameters
        ----------
        contexts: Collection[NodeContext]
            A collection of contexts that are to be summarised.

        Returns
        -------
        SystemSummary
            A summary of the system architecture.
        """
        summaries = [node.summarise() for node in contexts]
        node_to_summary = {s.fullname: s for s in summaries}
        return SystemSummary(node_to_summary)

    def _transform_state_to_nodeinfo(self, state: SystemState) -> t.Collection[NodeInfo]:
        """
        Produce information about ros keyed by node.

        Roswire state as information keyed by topic, which causes complex processing in
        rosdiscover which expects information keyed by node. We transform the roswire information
        so that it is keyed by node.

        Parameters
        ----------
        ros: ROS1
            State information from roswire

        Returns
        -------
        Collection[NodeInfo]
            A collection of nodes with publishers and subscribers and services attacbed
        """
        reorganized_nodes: t.Dict[str, NodeInfo] = dict()
        # Create the node placeholders
        for node_name in state.nodes:
            if node_name not in _NODES_TO_FILTER_OUT:
                node: NodeInfo = NodeInfo(
                    name=node_name[1:] if node_name.startswith('/') else node_name
                )
                reorganized_nodes[node_name] = node

        # Add in topics
        for topic in state.publishers:
            if topic not in _TOPICS_TO_FILTER_OUT:
                for node_name in state.publishers[topic]:
                    if node_name in reorganized_nodes:
                        reorganized_nodes[node_name].publishers.add(topic)

        for topic in state.subscribers:
            if topic not in _TOPICS_TO_FILTER_OUT:
                for node_name in state.subscribers[topic]:
                    if node_name in reorganized_nodes:
                        reorganized_nodes[node_name].subscribers.add(topic)

        # Add in services
        for service in state.services:
            if service.split('/')[-1] not in _SERVICES_TO_FILTER_OUT:
                for node_name in state.services[service]:
                    if node_name in reorganized_nodes:
                        reorganized_nodes[node_name].provides.add(service)

        return list(reorganized_nodes.values())

    def launch_from_config(self, sleep_time: float) -> t.Sequence[Popen]:
        # Eagerly check launch files exist - don't start any if one doesn't exist
        app_instance = self._app_instance
        for launch in self._config.launches:
            if not app_instance.files.exists(launch.filename):
                raise FileNotFoundError(launch.filename)

        processes = []

        for launch in self._config.launches:
            launch_script = self._generate_launch_script_on_host(launch)
            cmd = f"/bin/bash {launch_script}"
            process = app_instance.shell.popen(cmd)
            processes.append(process)
            time.sleep(sleep_time)
            if process.returncode and process.returncode != 0:
                raise ExecutionError(f"Could not run '{cmd}' on container.")
        return processes

    def _generate_launch_script_on_host(self, launch: Launch) -> str:
        script_path = None
        try:
            tmp = tempfile.NamedTemporaryFile(suffix=".sh", delete=False)
            with open(tmp.name, 'w') as script:
                for source in self._config.sources:
                    script.write(f"source {source}\n")
                for var, val in self._config.environment.items():
                    script.write(f"export {var}={val}\n")
                launch_cmd = f"roslaunch {launch.filename}"
                for arg in launch.get_argv():
                    launch_cmd += f" {arg}"
                logger.info(f"Launching with '{launch_cmd}")
                script.write(f"{launch_cmd}\n")
                script_path = script.name
            script_on_container = self._app_instance.files.mktemp(suffix='.sh')
            self._app_instance.files.copy_from_host(script_path, script_on_container)
            return script_on_container
        finally:
            if script_path:
                os.remove(script_path)
