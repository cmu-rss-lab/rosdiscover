# -*- coding: utf-8 -*-
import re
from typing import Dict, Optional

from loguru import logger

from .observer import Observer
from ..core import Topic
from ..interpreter import NodeContext, ParameterServer

_GOAL = re.compile(r'(.*)/goal')
_GOAL_FMT = re.compile(r'(.*)Goal')
_CANCEL = re.compile(r'(.*)/cancel')
_STATUS = re.compile(r'(.*)/status')
_FEEDBACK = re.compile(r'(.*)/feedback')
_FEEDBACK_FMT = re.compile(r'(.*)Feedback')
_RESULT = re.compile(r'(.*)/result')
_RESULT_FMT = re.compile(r'(.*)Result')

_NODES_TO_FILTER_OUT = {'/rosout'}
_TOPICS_TO_FILTER_OUT = {'/rosout', '/rosout_agg'}
_SERVICES_TO_FILTER_OUT = {'set_logger_level', 'get_loggers'}


class ActionCandidate:
    goal: Optional[Topic]
    cancel: Optional[Topic]
    status: Optional[Topic]
    feedback: Optional[Topic]
    result: Optional[Topic]

    def __init__(self,
                 action_name: str,
                 is_server: bool):
        self._action_name = action_name
        self._server = is_server
        self.goal = None
        self.cancel = None
        self.status = None
        self.feedback = None
        self.result = None

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
        if self._server:
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
    format_match = _GOAL_FMT.match(fmt)

    a_c = None
    if goal is not None and format_match is not None:
        action_name = goal.group(1)
        a_c = get_action_candidate(node, action_name, existing, not publishes)
        a_c.goal = Topic(topic, fmt, False)
        return True

    cancel = _CANCEL.match(topic)
    if cancel is not None and fmt == 'actionlib_msgs/GoalID':
        a_c = get_action_candidate(node, cancel.group(1), existing, not publishes)
        a_c.cancel = Topic(topic, fmt, False)
        return True

    status = _STATUS.match(topic)
    if status is not None and fmt == 'actionlib_msgs/GoalStatusArray':
        a_c = get_action_candidate(node, status.group(1), existing, publishes)
        a_c.status = Topic(topic, fmt, False)
        return True

    feedback = _FEEDBACK.match(topic)
    format_match = _FEEDBACK_FMT.match(fmt)
    if feedback is not None and format_match is not None:
        a_c = get_action_candidate(node, feedback.group(1), existing, publishes)
        a_c.feedback = Topic(topic, fmt, False)
        return True

    result = _RESULT.match(topic)
    format_match = _RESULT_FMT.match(fmt)
    if result is not None and format_match is not None:
        a_c = get_action_candidate(node, result.group(1), existing, publishes)
        a_c.result = Topic(topic, fmt, False)
        return True

    return False


def update_node_contexts_with_topics(nodecontexts, items, publishes,
                                     action_client_candidates,
                                     action_server_candidates, ros):
    for topic, nodes in items:
        if topic not in _TOPICS_TO_FILTER_OUT:
            if topic not in ros.topic_to_type:
                logger.error(f'Could not find the type for topic: {topic} in the AppInstance')
            else:
                fmt = ros.topic_to_type[topic]
                for node in [n for n in nodes if n not in _NODES_TO_FILTER_OUT]:
                    # Work out if this is a topic for a candidate action
                    could_be_action = action_candidate(node, topic, fmt,
                                                       publishes, action_server_candidates)
                    could_be_action = could_be_action or action_candidate(node, topic, fmt,
                                                                          publishes,
                                                                          action_client_candidates)

                    # If not, add the topic to the context
                    if not could_be_action:
                        if publishes:
                            nodecontexts[node].pub(topic, fmt)
                        else:
                            nodecontexts[node].sub(topic, fmt)


class ROS1Observer(Observer):

    def observe_and_summarise(self):
        try:
            nodecontexts: Dict[str, NodeContext] = {}
            with self._app_instance.ros1() as ros:
                nodes = [n for n in ros.nodes if n not in _NODES_TO_FILTER_OUT]
                info = ros.state
                # Create a context for each node
                # TODO: missing information?
                nodecontexts = dict((node,
                                     NodeContext(
                                         name=node[1:] if node.startswith('/') else node,
                                         namespace="",
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

                update_node_contexts_with_topics(nodecontexts, info.publishers.items(), True,
                                                 action_client_candidates,
                                                 action_server_candidates, ros)
                update_node_contexts_with_topics(nodecontexts, info.subscribers.items(), False,
                                                 action_client_candidates,
                                                 action_server_candidates, ros)

                for service, nodes in info.services.items():
                    if service.split('/')[-1] not in _SERVICES_TO_FILTER_OUT:
                        for node in nodes:
                            nodecontexts[node].provide(service,
                                                       ros.services[service].format.fullname)

                # Check if action candidates are complete (i.e., have all their topics)
                # and add action if they are, or add the topics back in if they're not
                for node, action_servers in action_server_candidates.items():
                    for action_server in action_servers.values():
                        if action_server.is_complete():
                            nodecontexts[node].action_server(action_server.name, action_server.fmt)
                        else:
                            logger.warning(f"Action {action_server._action_name} is incomplete")
                            logger.debug(
                                f"goal({action_server.goal}), cancel({action_server.cancel}), "
                                f"status({action_server.status}), feedback("
                                f"{action_server.feedback}), result({action_server.result}")
                            action_server.add_unfinished_to_context(nodecontexts[node])

                for node, action_clients in action_client_candidates.items():
                    for action_client in action_clients.values():
                        if action_client.is_complete():
                            nodecontexts[node].action_client(action_client.name, action_client.fmt)
                            logger.warning(f"Action {action_client._action_name} is incomplete")
                            logger.debug(
                                f"goal({action_client.goal}), cancel({action_client.cancel}), "
                                f"status({action_client.status}), feedback("
                                f"{action_client.feedback}), result({action_client.result}")

                    else:
                        action_client.add_unfinished_to_context(nodecontexts[node])
            self._nodes = nodecontexts
            return self.summarise()
        except TimeoutError:
            logger.exception("failed to connect to the ROS master. Is a robot running?")
