# -*- coding: utf-8 -*-
import re
from typing import Collection, Set

import attr
from loguru import logger
from roswire import ROS1

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
    publishers: Set[str] = attr.ib(default=set())
    subscribers: Set[str] = attr.ib(default=set())
    provides: Set[str] = attr.ib(default=set())

    @classmethod
    def filter_topics_for_action(cls,
                                 ros: ROS1, goal_related_topics: Set[str],
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
                    if cls.has_all_action_topics(action, fmt,
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
    def has_all_action_topics(cls,
                              action: str,
                              fmt: str,
                              ros: ROS1,
                              goal_related_topics: Collection[str],
                              result_related_topics: Collection[str]) -> bool:
        """Check that the non-goal related topics are in the right collections."""
        return cls.has_cancel(action, ros, goal_related_topics) and \
            cls.has_status(action, ros, result_related_topics) and \
            cls.has_feedback(action, fmt, ros, result_related_topics) and \
            cls.has_result(action, fmt, ros, result_related_topics)

    @classmethod
    def has_cancel(cls, action: str, ros: ROS1, topics: Collection[str]) -> bool:
        cancel = f"{action}/cancel"
        try:
            return cancel in topics and ros.topic_to_type[cancel] == "actionlib_msgs/GoalID"
        except KeyError:
            logger.error(f"Topic {cancel} does not have a type.")
            return False

    @classmethod
    def has_status(cls, action: str, ros: ROS1, topics: Collection[str]) -> bool:
        status = f"{action}/status"
        try:
            return status in topics and ros.topic_to_type[
                status] == "actionlib_msgs/GoalStatusArray"
        except KeyError:
            logger.error(f"Topic {status} does not have a type.")
            return False

    @classmethod
    def has_feedback(cls, action: str, fmt: str, ros: ROS1, topics: Collection[str]) -> bool:
        feedback = f"{action}/feedback"
        try:
            return feedback in topics and ros.topic_to_type[feedback] == f"{fmt}Feedback"
        except KeyError:
            logger.error(f"Topic {feedback} does not have a type.")
            return False

    @classmethod
    def has_result(cls, action: str, fmt: str, ros: ROS1, topics: Collection[str]) -> bool:
        result = f"{action}/result"
        try:
            return result in topics and ros.topic_to_type[result] == f"{fmt}Result"
        except KeyError:
            logger.error(f"Topic {result} does not have a type.")
            return False
