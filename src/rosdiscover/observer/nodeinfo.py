# -*- coding: utf-8 -*-
import re
from typing import Collection, Set

import attr
from loguru import logger
from roswire import AppInstance, ROS1

from rosdiscover.interpreter import NodeContext, ParameterServer

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
class NodeInfo:
    """Class for partial node information."""
    name: str
    publishers: Set[str] = attr.ib(factory=set)
    subscribers: Set[str] = attr.ib(factory=set)
    provides: Set[str] = attr.ib(factory=set)

    def _identify_action_servers(self, ros: ROS1) -> Collection[_Action]:
        """Identify the action servers for the node.

        Looks in the set of publishers for topics related to results, feedback, status and in the
        set of subscribers for cancel, goal. If these exist, then create an action server and
        remove the topics from the node.

        Returns
        -------
        Collection[_Action]
            A collection of information about each action server
        """
        return NodeInfo._filter_topics_for_action(ros, self.subscribers, self.publishers)

    def _identify_action_clients(self, ros: ROS1) -> Collection[_Action]:
        """Identify the action clients for the node.

        Looks in the set of subscribers for topics related to results, feedback, status and in the
        set of publishers for cancel, goal. If these exist, then create an action client and
        remove the topics from the node.

        Returns
        -------
        Collection[_Action]
            A collection of information about each action client
        """
        return NodeInfo._filter_topics_for_action(ros, self.publishers, self.subscribers)

    @classmethod
    def _filter_topics_for_action(cls,
                                  ros: ROS1,
                                  goal_related_topics: Set[str],
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
                    if cls._has_all_action_topics(action, fmt,
                                                  ros, goal_related_topics, result_related_topics):
                        # Remove the topics from the right collections and replace as an action
                        goal_related_topics.remove(f"{action}/goal")
                        goal_related_topics.remove(f"{action}/cancel")
                        result_related_topics.remove(f"{action}/status")
                        result_related_topics.remove(f"{action}/feedback")
                        result_related_topics.remove(f"{action}/result")
                        actions.append(_Action(action, fmt))
        return actions

    @classmethod
    def _has_all_action_topics(cls,
                               action: str,
                               fmt: str,
                               ros: ROS1,
                               goal_related_topics: Collection[str],
                               result_related_topics: Collection[str]) -> bool:
        """Check that the non-goal related topics are in the right collections."""
        return cls._has_cancel(action, ros, goal_related_topics) and \
            cls._has_status(action, ros, result_related_topics) and \
            cls._has_feedback(action, fmt, ros, result_related_topics) and \
            cls._has_result(action, fmt, ros, result_related_topics)

    @classmethod
    def _has_cancel(cls, action: str, ros: ROS1, topics: Collection[str]) -> bool:
        cancel = f"{action}/cancel"
        try:
            return cancel in topics and ros.topic_to_type[cancel] == "actionlib_msgs/GoalID"
        except KeyError:
            logger.error(f"Topic {cancel} does not have a type.")
            return False

    @classmethod
    def _has_status(cls, action: str, ros: ROS1, topics: Collection[str]) -> bool:
        status = f"{action}/status"
        try:
            return status in topics and ros.topic_to_type[
                status] == "actionlib_msgs/GoalStatusArray"
        except KeyError:
            logger.error(f"Topic {status} does not have a type.")
            return False

    @classmethod
    def _has_feedback(cls, action: str, fmt: str, ros: ROS1, topics: Collection[str]) -> bool:
        feedback = f"{action}/feedback"
        try:
            return feedback in topics and ros.topic_to_type[feedback] == f"{fmt}Feedback"
        except KeyError:
            logger.error(f"Topic {feedback} does not have a type.")
            return False

    @classmethod
    def _has_result(cls, action: str, fmt: str, ros: ROS1, topics: Collection[str]) -> bool:
        result = f"{action}/result"
        try:
            return result in topics and ros.topic_to_type[result] == f"{fmt}Result"
        except KeyError:
            logger.error(f"Topic {result} does not have a type.")
            return False

    def make_node_context(self, ros: ROS1, app_instance: AppInstance) -> NodeContext:
        """
        Construct a NodeContext used in rosdiscover from the processed information from roswire.
        NodeContext is required by rosdiscover to produce the system summary.
        See Also :class:`rosdiscover.interpreter.context.NodeContext`

        Parameters
        ----------
        node: NodeInfo
            The processed node information from roswire
        ros: ROS1
            Information about the state (used for getting types)
        app_instance: AppInstance
            The instance the context will be associated with

        Returns
        -------
            A NodeContext
        """
        nodecontext = NodeContext(
            name=self.name,
            namespace="",
            kind="",
            package="unknown",
            args="unknown",
            remappings={},
            launch_filename="unknown",
            app=app_instance,
            files=app_instance.files,
            params=ParameterServer()
        )

        # Process the action servers and clients, which mutates that publishers and subscribers
        act_srvrs = self._identify_action_servers(ros)
        act_clnts = self._identify_action_clients(ros)

        for action in act_srvrs:
            nodecontext.action_server(action.name, action.fmt)

        for action in act_clnts:
            nodecontext.action_client(action.name, action.fmt)

        # Process the topics
        for topic in self.publishers:
            try:
                nodecontext.pub(topic, ros.topic_to_type[topic])
            except KeyError:
                logger.error(f"Topic {topic} does not have a type.")

        for topic in self.subscribers:
            try:
                nodecontext.sub(topic, ros.topic_to_type[topic])
            except KeyError:
                logger.error(f"Topic {topic} does not have a type.")

        # Process the service information. Note, ROS1 state has no knowledge of clients
        for service in self.provides:
            nodecontext.provide(service, ros.services[service].format.fullname)

        return nodecontext
