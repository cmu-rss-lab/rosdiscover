# -*- coding: utf-8 -*-
__all__ = ('ROS1Observer',)

import os
from typing import Collection, Dict

from dockerblade.popen import Popen
from loguru import logger
from roswire.common import SystemState

from .nodeinfo import NodeInfo
from .observer import Observer
from ..interpreter import NodeContext, SystemSummary

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

    def execute_script(self, path_on_host: str) -> Popen:
        if not os.path.exists(path_on_host):
            raise FileNotFoundError(f"'{path_on_host}' not found.")
        assert self._app_instance is not None

        path_on_container = self._app_instance.files.mktemp('.sh')
        self._app_instance.files.copy_from_host(path_on_host, path_on_container)

        cmd = f"bash {path_on_container}"

        logger.debug(f"Running the script in the container: {cmd}")
        process = self._app_instance.shell.popen(cmd)
        return process
