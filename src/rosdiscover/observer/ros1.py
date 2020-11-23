# -*- coding: utf-8 -*-
import re
from typing import Dict, Optional, Tuple

from attr import attr
from loguru import logger

from .observer import Observer
from ..core import Topic
from ..interpreter import NodeContext, ParameterServer

_GOAL = re.compile(r'(.*)/goal$')
_GOAL_FMT = re.compile(r'(.*)Goal$')
_CANCEL = re.compile(r'(.*)/cancel$')
_STATUS = re.compile(r'(.*)/status$')
_FEEDBACK = re.compile(r'(.*)/feedback$')
_RESULT = re.compile(r'(.*)result$')

_ACTION_FILTER_PATTERNS = [_GOAL, _CANCEL, _STATUS, _FEEDBACK, _RESULT]


class ActionCandidate:
    goal: Topic
    cancel: Topic
    status: Topic
    feedback: Topic
    result: Topic

    def __init__(self,
                 action_name: str,
                 is_server: bool):
        self._action_name = action_name
        self._server = is_server

    @property
    def name(self):
        return self._action_name

    @property
    def fmt(self):
        assert self.is_complete()
        return _GOAL_FMT.match(self.goal.format).group(1)

    def is_complete(self):
        """Check if all information is there for an action.

        Returns
        ------_
        bool
            Whether the action is complete
        """
        return self.goal is not None and self.cancel is not None and self.status is not None \
               and self.feedback is not None and self.result is not None

    def add(self,
            nc: NodeContext,
            topic: Topic,
            sub_if_server):
        """Add the topic back into the context.

        Parameters
        ----------
        nc: NodeContext
            The node context to add the topic to
        topic: Topic
            The topic to add to the context
        sub_if_server: bool
            Indicate wheter to subscribe or publish if this was expected to be a server
            or a client
        """
        if self.server:
            if sub_if_server:
                nc.sub(topic.name, topic.format)
            else:
                nc.pub(topic.name, topic.format)
        else:
            if sub_if_server:
                nc.pub(topic.name, topic.format)
            else:
                nc.sub(topic.name, topic.format)

    def add_unfinished_to_context(self, nc: NodeContext):
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
    the action if it is meant to be.

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
    goal = _GOAL.match(topic)
    a_c = None
    if goal is not None and fmt == goal.group(1) + "Goal":
        action_name = goal.group(1)
        a_c = get_action_candidate(node, action_name, existing, not publishes)
        a_c.goal = Topic(topic, fmt)
        return True

    cancel = _CANCEL.match(topic)
    if cancel is not None and fmt == 'actionlib_msgs/GoalID':
        a_c = get_action_candidate(node, cancel.group(1), existing, not publishes)
        a_c.cancel = Topic(topic, fmt)
        return True

    status = _STATUS.match(topic)
    if status is not None and fmt == 'actionlib_msgs/GoalStatusArray':
        a_c = get_action_candidate(node, status.group(1), existing, publishes)
        a_c.status = Topic(topic, fmt)
        return True

    feedback = _FEEDBACK.match(topic)
    if feedback is not None and fmt == f"{feedback.group(1)}Feedback":
        a_c = get_action_candidate(node, feedback.group(1), existing, publishes)
        a_c.feedback = Topic(topic, fmt)
        return True

    result = _RESULT.match(topic)
    if result is not None and fmt == f"{result.group(1)}Result":
        a_c = get_action_candidate(node, result.group(1), existing, publishes)
        a_c.result = Topic(topic, fmt)
        return True

    return False


class ROS1Observer(Observer):

    def observe_and_summarise(self):
        try:
            nodecontexts: Dict[str, NodeContext] = {}
            with self._app_instance.ros1() as ros:
                nodes = ros.nodes
                info = ros.state
                # Create a context for each node
                # TODO: missing information?
                nodecontexts = dict((node,
                                     NodeContext(name=node,
                                                 namespace=node,
                                                 kind="",
                                                 package="unknown",
                                                 args="unknown",
                                                 remappings={},
                                                 launch_filename="unknown",
                                                 app=self._app_instance,
                                                 files=self._app_instance.files,
                                                 params=ParameterServer(),
                                                 ))
                                    for node in nodes)
                # Places to store bits of actions, which only appear as topics
                action_server_candidates = dict()
                action_client_candidates = dict()

                for topic, nodes in info.publishers.items():
                    fmt = ros.topic_to_type[topic]
                    for node in nodes:
                        # Work out if this is a topic for a candidate action
                        add_topic = action_candidate(node, topic, fmt,
                                                          True, action_server_candidates)
                        add_topic |= action_candidate(node, topic, fmt,
                                                           True, action_client_candidates)

                        # If not, add the topic to the context
                        if add_topic:
                            nodecontexts[node].pub(topic, fmt)

                for topic, nodes in info.subscribers.items():
                    fmt = ros.topic_to_type[topic]
                    for node in nodes:
                        # Work out if this is a topic for a candidate action
                        add_topic = action_candidate(node, topic, fmt,
                                                          False, action_server_candidates)
                        add_topic |= action_candidate(node, topic, fmt,
                                                           False, action_client_candidates)
                        # If not, add the topic to the context
                        if add_topic:
                            nodecontexts[node].sub(topic, fmt)

                for service, nodes in info.services.items():
                    for node in nodes:
                        nodecontexts[node].provide(service, "Not known")

                # Check if action candidates are complete (i.e., have all their topics)
                # and add action if they are, or add the topics back in if they're not
                for name, action_server in action_server_candidates.items():
                    if action_server.is_complete():
                        nodecontexts[node].action_server(action_server.name, action_server.fmt)
                    else:
                        action_server.add_unfinished_to_context(nodecontexts[node])

                for name, action_client in action_client_candidates.items():
                    if action_client.is_complete():
                        nodecontexts[node].action_client(action_client.name, action_client.fmt)
                    else:
                        action_client.add_unfinished_to_context(nodecontexts[node])
            self._nodes = nodecontexts
            return self.summarise()
        except TimeoutError:
            logger.exception("failed to connect to the ROS master. Is a robot running?")
