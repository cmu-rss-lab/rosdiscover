# -*- coding: utf-8 -*-
__all__ = ('ROS1Observer',)

import typing
from typing import Collection, Dict, Set

from loguru import logger
from roswire import AppInstance, ROS1
from roswire.common import SystemState

from .nodeinfo import NodeInfo
from .observer import Observer
from ..interpreter import NodeContext, ParameterServer, SystemSummary

if typing.TYPE_CHECKING:
    from ..config import Config

_NODES_TO_FILTER_OUT = ('/rosout',)
_TOPICS_TO_FILTER_OUT = ('/rosout', '/rosout_agg')
_SERVICES_TO_FILTER_OUT = ('set_logger_level', 'get_loggers')


class ROS1Observer(Observer):

    def __init__(self, app: AppInstance, config: 'Config') -> None:
        super().__init__(app, config)

    def observe(self) -> SystemSummary:
        """Observe the state of the running system and produce a summery of the archtiecture."""
        nodecontexts: Set[NodeContext] = set()
        with self._app_instance.ros1() as ros:
            info = ros.state
            nodes = self._transform_info(info)
            for node in nodes:
                nodecontext = self._make_node_context(node, ros)
                nodecontexts.add(nodecontext)

        return self._summarise(nodecontexts)

    def _summarise(self, contexts: Collection[NodeContext]) -> SystemSummary:
        """Produces an immutable description of the system architecture.

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

    def _make_node_context(self, node: NodeInfo, ros: ROS1) -> NodeContext:
        """
        Construct a NodeContext used in rosdiscover from the processed information from roswire.

        Parameters
        ----------
        node: NodeInfo
            The processed node information from roswire
        ros: ROS1
            Information about the state (used for getting types)

        Returns
        -------
            A NodeContext
        """
        nodecontext = NodeContext(
            name=node.name,
            namespace="",
            kind="",
            package="unknown",
            args="unknown",
            remappings={},
            launch_filename="unknown",
            app=self._app_instance,
            files=self._app_instance.files,
            params=ParameterServer()
        )

        # Process the action servers and clients, which mutates that publishers and subscribers
        act_srvrs = NodeInfo.filter_topics_for_action(ros, node.subscribers, node.publishers)
        act_clnts = NodeInfo.filter_topics_for_action(ros, node.publishers, node.subscribers)

        for action in act_srvrs:
            nodecontext.action_server(action.name, action.fmt)

        for action in act_clnts:
            nodecontext.action_client(action.name, action.fmt)

        # Process the topics
        for t in node.publishers:
            try:
                nodecontext.pub(t, ros.topic_to_type[t])
            except KeyError:
                logger.error(f"Topic {t} does not have a type.")

        for t in node.subscribers:
            try:
                nodecontext.sub(t, ros.topic_to_type[t])
            except KeyError:
                logger.error(f"Topic {t} does not have a type.")

        # Process the service information. Note, ROS1 state has no knowledge of clients
        for s in node.provides:
            nodecontext.provide(t, ros.services[s].format.fullname)

        return nodecontext

    def _transform_info(self, ros: SystemState) -> Collection[NodeInfo]:
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
        for n in ros.nodes:
            if n not in _NODES_TO_FILTER_OUT:
                node: NodeInfo = NodeInfo(name=n[1:] if n.startswith('/') else n)
                reorganized_nodes[n] = node

        # Add in topics
        for topic, nodes in ros.publishers.items():
            if topic not in _TOPICS_TO_FILTER_OUT:
                for n in nodes:
                    if n in reorganized_nodes:
                        reorganized_nodes[n].publishers.add(topic)

        for topic, nodes in ros.subscribers.items():
            if topic not in _TOPICS_TO_FILTER_OUT:
                for n in nodes:
                    if n in reorganized_nodes:
                        reorganized_nodes[n].subscribers.add(topic)

        # Add in services
        for service, nodes in ros.services.items():
            if service.split('/')[-1] not in _SERVICES_TO_FILTER_OUT:
                for n in nodes:
                    if n in reorganized_nodes:
                        reorganized_nodes[n].provides.add(service)

        return reorganized_nodes.values()
