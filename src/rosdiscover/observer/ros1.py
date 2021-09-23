# -*- coding: utf-8 -*-
__all__ = ('ROS1Observer',)

import os
import tempfile
import time
from typing import Collection, Dict

from loguru import logger
from roswire.common import SystemState

from .nodeinfo import NodeInfo
from .observer import Observer
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

    def _summarise(self, contexts: Collection[NodeContext]) -> SystemSummary:
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

    def _transform_state_to_nodeinfo(self, state: SystemState) -> Collection[NodeInfo]:
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
        reorganized_nodes: Dict[str, NodeInfo] = dict()
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

    def launch_from_config(self, sleep_time: float) -> int:
        # Eagerly check launch files exist - don't start any if one doesn't exist
        app_instance = self._app_instance
        for launch in self._config.launches:
            if not app_instance.files.exists(launch.filename):
                raise FileNotFoundError(launch.filename)

        i = 0
        for launch in self._config.launches:
            launch_script = self._generate_launch_script_on_host(launch)
            cmd = f"bash {launch_script}"
            completed = app_instance.shell.run(cmd)
            if completed.returncode != 0:
                return completed.returncode
            if i < len(self._config.launches)-1:
                time.sleep(sleep_time)
            i += 1

    def _generate_launch_script_on_host(self, launch: Launch) -> str:
        script_path = None
        try:
            with tempfile.NamedTemporaryFile(suffix=".sh", delete=False) as script:
                for source in self._config.sources:
                    script.write(f"source {source}\n")
                script.write(f"roslaunch {launch.filename} {launch.get_argv()}")
                script_path = script.name
            script_on_container = self._app_instance.files.mktemp(suffix='.sh')
            self._app_instance.files.copy_from_host(script_path, script_on_container)
            return script_on_container
        finally:
            if script_path:
                os.remove(script_path)



