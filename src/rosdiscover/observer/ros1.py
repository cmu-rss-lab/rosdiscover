# -*- coding: utf-8 -*-
from typing import Callable, Collection, Dict, Mapping

from loguru import logger
from roswire import AppInstance

from .observer import Observer
from .ros1_actions import ActionCandidate, separate_topics_from_action
from .. import Config
from ..interpreter import NodeContext, ParameterServer, SystemSummary

_NODES_TO_FILTER_OUT = {'/rosout'}
_TOPICS_TO_FILTER_OUT = {'/rosout', '/rosout_agg'}
_SERVICES_TO_FILTER_OUT = {'set_logger_level', 'get_loggers'}


class ROS1Observer(Observer):

    def __init__(self, app: AppInstance, config: Config):
        super().__init__(app, config)

    def observe_and_summarise(self) -> SystemSummary:
        """
        Takes a snapshot of the ROS system running in the container referred to by
        ``self.app_instance`` and returns the architecture summary for that system.

        Returns
        -------
        The summary of the instantaneous state of the ROS system

        Raises
        ------
        TimeoutException
            If a connection cannot be made to the container
        """
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
                action_server_candidates: Dict[str, Dict[str, ActionCandidate]] = dict()
                action_client_candidates: Dict[str, Dict[str, ActionCandidate]] = dict()
                # Only include topics that we care about
                significant_publishers = ROS1Observer._filter_out_topics_and_nodes(info.publishers)
                significant_subscribers = ROS1Observer._filter_out_topics_and_nodes(
                    info.subscribers)

                # Process the topics
                separate_topics_from_action(nodecontexts,
                                            significant_publishers, True,
                                            action_client_candidates,
                                            action_server_candidates, ros)
                separate_topics_from_action(nodecontexts,
                                            significant_subscribers, False,
                                            action_client_candidates,
                                            action_server_candidates, ros)

                # Process the services
                for service, nodes in info.services.items():
                    if service.split('/')[-1] not in _SERVICES_TO_FILTER_OUT:
                        for node in nodes:
                            nodecontexts[node].provide(service,
                                                       ros.services[service].format.fullname)

                # Check if action candidates are complete (i.e., have all their topics)
                # and add action if they are, or add the topics back in if they're not
                self.process_action_candidates(action_server_candidates,
                                               lambda node: nodecontexts[node].action_server,
                                               nodecontexts)
                self.process_action_candidates(action_client_candidates,
                                               lambda node: nodecontexts[node].action_client,
                                               nodecontexts)

            self._nodes = nodecontexts
            return self.summarise()
        except TimeoutError:
            logger.exception("failed to connect to the ROS master. Is a robot running?")
            raise

    def process_action_candidates(self,
                                  action_candidates: Dict[str, Dict[str, ActionCandidate]],
                                  add_function: Callable[[str], Callable[[str, str], None]],
                                  nodecontexts: Dict[str, NodeContext]) -> None:
        """
        Processes action candidates, adding actions if the candidates are complete or adding
        the topics back in if the action is incomplete.

        Parameters
        ----------
        action_candidates: Dict[str, Dict[str, ActionCandidate]]
            The set of candidates to process, keyed by node and action name
        add_function: Callable[[str], Callable[[str, str], None]]
            The function used to add the action to a node (either ``action_client`` or
            ``action_server``
        nodecontexts: Dict[str, NodeContext]
            The node contexts to add actions parts to
        """
        for node, candidates in action_candidates.items():
            for action_candidate in candidates.values():
                if action_candidate.is_complete():
                    add_function(node)(action_candidate.name, action_candidate.fmt)
                else:
                    logger.warning(f"Action {action_candidate.name} is incomplete")
                    logger.debug(
                        f"goal({action_candidate.goal}), cancel({action_candidate.cancel}), "
                        f"status({action_candidate.status}), feedback("
                        f"{action_candidate.feedback}), result({action_candidate.result}")
                    action_candidate.add_unfinished_to_context(nodecontexts[node])

    @staticmethod
    def _filter_out_topics_and_nodes(
            topics_and_nodes: Mapping[str, Collection[str]]
    ) -> Dict[str, Collection[str]]:
        filtered: Dict[str, Collection[str]] = dict()
        for topic, nodes in topics_and_nodes.items():
            if topic not in _TOPICS_TO_FILTER_OUT:
                filtered[topic] = [node for node in nodes if node not in _NODES_TO_FILTER_OUT]
        return filtered
