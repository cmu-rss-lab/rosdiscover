# -*- coding: utf-8 -*-
__all__ = ('ROS1Observer',)

import re
from typing import Collection, Set, Dict

import attr
import typing
from loguru import logger
from roswire import AppInstance, ROS1
from roswire.common import SystemState

from .observer import Observer
from ..interpreter import NodeContext, ParameterServer, SystemSummary

if typing.TYPE_CHECKING:
    from ..config import Config

_NODES_TO_FILTER_OUT = ('/rosout',)
_TOPICS_TO_FILTER_OUT = ('/rosout', '/rosout_agg')
_SERVICES_TO_FILTER_OUT = ('set_logger_level', 'get_loggers')

_GOAL = re.compile(r'(.*)/goal')
_GOAL_FMT = re.compile(r'(.*)Goal')
_CANCEL = re.compile(r'(.*)/cancel')
_STATUS = re.compile(r'(.*)/status')
_FEEDBACK = re.compile(r'(.*)/feedback')
_FEEDBACK_FMT = re.compile(r'(.*)Feedback')
_RESULT = re.compile(r'(.*)/result')
_RESULT_FMT = re.compile(r'(.*)Result')


@attr.s(slots=True, auto_attribs=True)
class _Action:
    """Class for action information."""
    name: str
    fmt: str


@attr.s(slots=True, auto_attribs=True)
class _Node:
    """Class for partial node information."""
    name: str
    publishers: Set[str] = attr.ib(default=set())
    subscribers: Set[str] = attr.ib(default=set())
    provides: Set[str] = attr.ib(default=set())

    @staticmethod
    def filter_topics_for_action(ros: ROS1, goal_related_topics: Set[str],
                                 result_related_topics: Set[str]) -> Collection[_Action]:
        """
        Filter the topics in :code:`goal_related_topics` and `:code:result_related_topics` to
        pull out action-related topics.

        Parameters
        ----------
        ros: ROS1
            Information about the state of the ROS system.
        goal_related_topics: Set[str]
            The topics where the action goal is meant to be. Servers will subscribe to this,
            clients will publish.
        result_related_topics: Set[str]
            The topics where the action result should be. Servers will publish, clients will
            subscribe.

        Returns
        -------
        A collection of actions that were removed from the topic collections.
        """
        actions = []
        # Copy the topic set containing the goal because we are going to mutate it
        topic_copy = set(goal_related_topics)

        # Process the nodes for topics that are action related
        for topic in topic_copy:
            goal_match = _GOAL.match(topic)
            if goal_match:  # The topic might be a action goal
                fmt_match = _GOAL_FMT.match(ros.topic_to_type[topic])
                if fmt_match:  # The topic is an action goal
                    # Have the right goal and format matches. Check if other topics are there
                    action = goal_match.group(1)
                    fmt = fmt_match.group(1)
                    if _Node.has_all_action_topics(action, fmt,
                                                   ros, goal_related_topics, result_related_topics):
                        # Remove the topics from the right collections and replace as an action
                        goal_related_topics.remove(f"{action}/goal")
                        goal_related_topics.remove(f"{action}/cancel")
                        result_related_topics.remove(f"{action}/status")
                        result_related_topics.remove(f"{action}/feedback")
                        result_related_topics.remove(f"{action}/result")
                        actions.append(_Action(action, fmt))
        return actions

    @staticmethod
    def has_all_action_topics(action: str,
                              fmt: str,
                              ros: ROS1,
                              goal_related_topics: Collection[str],
                              result_related_topics: Collection[str]) -> bool:
        """Check that the non-goal related topics are in the right collections."""
        return _Node.has_cancel(action, ros, goal_related_topics) and \
            _Node.has_status(action, ros, result_related_topics) and \
            _Node.has_feedback(action, fmt, ros, result_related_topics) and \
            _Node.has_result(action, fmt, ros, result_related_topics)

    @staticmethod
    def has_cancel(action: str, ros: ROS1, topics: Collection[str]) -> bool:
        cancel = f"{action}/cancel"
        try:
            return cancel in topics and ros.topic_to_type[cancel] == "actionlib_msgs/GoalID"
        except KeyError:
            logger.error(f"Topic {cancel} does not have a type.")
            return False

    @staticmethod
    def has_status(action: str, ros: ROS1, topics: Collection[str]) -> bool:
        status = f"{action}/status"
        try:
            return status in topics and ros.topic_to_type[
                status] == "actionlib_msgs/GoalStatusArray"
        except KeyError:
            logger.error(f"Topic {status} does not have a type.")
            return False

    @staticmethod
    def has_feedback(action: str, fmt: str, ros: ROS1, topics: Collection[str]) -> bool:
        feedback = f"{action}/feedback"
        try:
            return feedback in topics and ros.topic_to_type[feedback] == f"{fmt}Feedback"
        except KeyError:
            logger.error(f"Topic {feedback} does not have a type.")
            return False

    @staticmethod
    def has_result(action: str, fmt: str, ros: ROS1, topics: Collection[str]) -> bool:
        result = f"{action}/result"
        try:
            return result in topics and ros.topic_to_type[result] == f"{fmt}Result"
        except KeyError:
            logger.error(f"Topic {result} does not have a type.")
            return False


class ROS1Observer(Observer):

    def __init__(self, app: AppInstance, config: 'Config'):
        super().__init__(app, config)

    def observe_and_summarise(self) -> SystemSummary:
        with self._app_instance.ros1() as ros:
            info = ros.state
            nodes = self._transform_info(info)
            for node in nodes:
                nodecontext = self._make_node_context(node, ros)
                self._nodes[nodecontext.name] = nodecontext
            return self.summarise()

    def _make_node_context(self, node: _Node, ros: ROS1) -> NodeContext:
        """
        Construct a NodeContext used in rosdiscover from the processed information from roswire.

        Parameters
        ----------
        node: _Node
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
        act_srvrs = _Node.filter_topics_for_action(ros, node.subscribers, node.publishers)
        act_clnts = _Node.filter_topics_for_action(ros, node.publishers, node.subscribers)

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

    def _transform_info(self, ros: SystemState) -> Collection[_Node]:
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
        Collection[_Node]
            A collection of nodes with publishers and subscribers and services attacbed
        """
        reorganized_nodes: Dict[str, _Node] = dict()

        # Create the node placeholders
        for n in ros.nodes:
            if n not in _NODES_TO_FILTER_OUT:
                node: _Node = _Node(name=n[1:] if n.startswith('/') else n)
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
