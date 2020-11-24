# -*- coding: utf-8 -*-
__all__ = ("separate_topics_from_action", "ActionCandidate")

import re
from typing import Collection, Dict

import attr
from loguru import logger
from roswire import ROS1

from ..core import Topic
from ..interpreter import NodeContext

_GOAL = re.compile(r'(.*)/goal')
_GOAL_FMT = re.compile(r'(.*)Goal')
_CANCEL = re.compile(r'(.*)/cancel')
_STATUS = re.compile(r'(.*)/status')
_FEEDBACK = re.compile(r'(.*)/feedback')
_FEEDBACK_FMT = re.compile(r'(.*)Feedback')
_RESULT = re.compile(r'(.*)/result')
_RESULT_FMT = re.compile(r'(.*)Result')


@attr.s(slots=True, auto_attribs=True)
class ActionCandidate:
    """
    Because of the way that the SystemState is organized with topics, topics associated with
    actions will be encountered in random order. This class keeps track of all the topics
    associated with a candidate action server (or action client) as the topics are encountered.
    """
    name: str
    is_server: bool
    goal: Topic = attr.ib(init=False)
    cancel: Topic = attr.ib(init=False)
    status: Topic = attr.ib(init=False)
    feedback: Topic = attr.ib(init=False)
    result: Topic = attr.ib(init=False)

    @property
    def fmt(self) -> str:
        assert self.is_complete()
        match = _GOAL_FMT.match(self.goal.format)
        assert match is not None
        return match.group(1)

    def is_complete(self) -> bool:
        """Check if all information is there for an action.

        Returns
        -------
        bool
            Whether the action is complete
        """
        has_goal = self.goal is not None
        has_cancel = self.cancel is not None
        has_status = self.status is not None
        has_feedback = self.feedback is not None
        has_result = self.result is not None
        return has_goal and has_cancel and has_status and has_feedback and has_result

    def add(self,
            context: NodeContext,
            topic: Topic,
            sub_if_server: bool) -> None:
        """Add the topic back into the context.

        Parameters
        ----------
        context: NodeContext
            The node context to add the topic to
        topic: Topic
            The topic to add to the context
        sub_if_server: bool
            Indicate wheter to subscribe or publish if this was expected to be a server
            or a client
        """
        if self.is_server:  # i.e., the candidate is an action server
            if sub_if_server:  # i.e., action servers subscribe to this topic
                context.sub(topic.name, topic.format)
            else:  # i.e., action servers publish this topic
                context.pub(topic.name, topic.format)
        else:  # i.e., the candidate is an action client
            if sub_if_server:  # i.e., action clients publish this topic
                context.pub(topic.name, topic.format)
            else:  # i.e., action clients subscribe to this topic
                context.sub(topic.name, topic.format)

    def add_unfinished_to_context(self, nc: NodeContext) -> None:
        """Add topics from incomplete action server/client to node context

        Parameters
        ----------
        nc: NodeContext
            The node to add the topics to
        """
        assert not self.is_complete()
        if self.goal is not None:
            self.add(nc, self.goal, True)
        if self.cancel is not None:
            self.add(nc, self.cancel, True)
        if self.status is not None:
            self.add(nc, self.status, False)
        if self.feedback is not None:
            self.add(nc, self.feedback, False)
        if self.result is not None:
            self.add(nc, self.result, False)


def get_action_candidate(node: str,
                         action_name: str,
                         existing: Dict[str, Dict[str, ActionCandidate]],
                         is_server: bool) -> ActionCandidate:
    """Gets the action candidate if it exsits

    Parameters
    ----------
    node: str
        The name of the node that the action might belong to
    action_name: str
        The name of the action to look for or create
    existing: Dict[str, Dict[str, ActionCandidate]]
        The existing known actions. This is updated if a new candidate is created
    is_server:
        Whether this should be an action server

    Returns
    -------
    ActionCandidate
        The new or existing action candidate

    """
    if node in existing:
        if action_name in existing[node]:
            a_s = existing[node][action_name]
        else:
            a_s = ActionCandidate(action_name, is_server)
            existing[node][action_name] = a_s
    else:
        a_s = ActionCandidate(action_name, is_server)
        existing[node] = {action_name: a_s}
    return a_s


def action_candidate(node: str,
                     topic: str,
                     fmt: str,
                     publishes: bool,
                     existing: Dict[str, Dict[str, ActionCandidate]]):
    """
    Check if the topic might belong to an action. The parameter
    :code:'existing' is updated with a new candidate if it is created. The topic is added to
    the action if it patches one of the action patterns.

    Parameters
    ----------
    node: str
        The name of the node that the action might be associated with.
    topic: str
        The name of the topic that could be part of an action.
    fmt: str
        The topic format associated with the topic.
    publishes: bool
        Indicates whether the topic is published by the node (subscribed otherwise).
    existing: Dict[str, Dict[str, ActionCandidate]]
        The existing candidates. This will be updated if the topic might be part of
        an action and the candidate isn't already known.
    """
    a_c = None

    # Check to see if the topic might be part of an action, and assign it to the right
    # place in the action candidate if it is.
    # Action servers subscribe to topics: goal, and cancel
    #                publish to topics: status, feedback, result
    # Action clients subscribe/publish to the opposite

    # Is this a goal topic?
    goal = _GOAL.match(topic)
    format_match = _GOAL_FMT.match(fmt)
    if goal is not None and format_match is not None:
        action_name = goal.group(1)
        a_c = get_action_candidate(node, action_name, existing, not publishes)
        a_c.goal = Topic(topic, fmt, False)
        return True

    # Is this a cancel topic?
    cancel = _CANCEL.match(topic)
    if cancel is not None and fmt == 'actionlib_msgs/GoalID':
        a_c = get_action_candidate(node, cancel.group(1), existing, not publishes)
        a_c.cancel = Topic(topic, fmt, False)
        return True

    # Is this a status topic?
    status = _STATUS.match(topic)
    if status is not None and fmt == 'actionlib_msgs/GoalStatusArray':
        a_c = get_action_candidate(node, status.group(1), existing, publishes)
        a_c.status = Topic(topic, fmt, False)
        return True

    # is this a feedback topic?
    feedback = _FEEDBACK.match(topic)
    format_match = _FEEDBACK_FMT.match(fmt)
    if feedback is not None and format_match is not None:
        a_c = get_action_candidate(node, feedback.group(1), existing, publishes)
        a_c.feedback = Topic(topic, fmt, False)
        return True

    # Is this a result topic?
    result = _RESULT.match(topic)
    format_match = _RESULT_FMT.match(fmt)
    if result is not None and format_match is not None:
        a_c = get_action_candidate(node, result.group(1), existing, publishes)
        a_c.result = Topic(topic, fmt, False)
        return True

    # It's not any kind of action related topic
    return False


def separate_topics_from_action(nodecontexts: Dict[str, NodeContext],
                                topics: Dict[str, Collection[str]],
                                publishing: bool,
                                action_client_candidates: Dict[str, Dict[str, ActionCandidate]],
                                action_server_candidates: Dict[str, Dict[str, ActionCandidate]],
                                ros: ROS1) -> None:
    """
    Separates the ordinary topics from the topics that might be part of action servers or
    clients. If the topic might be part of an action server/client, then it is added to an
    action candidate and stored in either ``action_client_candidates`` or
    ``action_server_candidates`` depending on whether the topic is a publisher or subscriber
    (as indicated by ``publishing``). Otherwise, it is added to the associated ``NodeContext``.

    Parameters
    ----------
    nodecontexts: Dict[str, NodeContext]
        The known node contexts, keyed by the node name
    topics: Iterable[Tuple[str, Collection[str]]]
        The collection of topics (either publishers or subscribers) retrieved from the
        SystemState. Each element is a topic name and a list of nodes that publish that topic
    publishing: bool
        Are the topics published or subscribed to
    action_client_candidates: Dict[str, Dict[str, ActionCandidate]]
        The collection of action client candidates, keyed by node name
    action_server_candidates: Dict[str, Dict[str, ActionCandidate]]
        The collection of action server candidates, keyed by node name
    ros: ROS1
        Access the underlying connection to ROS on the container
    """
    for topic, nodes in topics:
        if topic not in ros.topic_to_type:
            logger.error(f'Could not find the type for topic: {topic} in the AppInstance')
        else:
            fmt = ros.topic_to_type[topic]
            for node in nodes:
                # Work out if this is a topic for a candidate action
                maybe_action_server = action_candidate(node, topic, fmt,
                                                       publishing,
                                                       action_server_candidates)
                maybe_action_client = action_candidate(node,
                                                       topic,
                                                       fmt,
                                                       publishing,
                                                       action_client_candidates)
                maybe_action = maybe_action_client or maybe_action_server
                # If not a possible action, add the topic to the context
                if not maybe_action:
                    if publishing:
                        nodecontexts[node].pub(topic, fmt)
                    else:
                        nodecontexts[node].sub(topic, fmt)
