# -*- coding: utf-8 -*-
__all__ = ("Observer",)
import contextlib
import os
from typing import Dict, Iterator

from roswire import App, AppInstance, ROSVersion

from .ros1 import ROS1ObserverConnection
from .ros2 import ROS2ObserverConnection
from ..config import Config
from ..interpreter import NodeContext, SystemSummary

_DEFAULT_URL = os.environ.get("DOCKER_HOST", "unix://var/run/docker.sock")


class Observer:

    @classmethod
    @contextlib.contextmanager
    def for_container(cls,
                      container: str,
                      config: Config,
                      ) -> Iterator['Observer']:
        """Constructs and interpreter for a given running container"""
        app: App = App(config, None)
        instance = app.attach(container, require_description=False)
        yield Observer(instance, config)

    def __init__(self, app: AppInstance, config: Config):
        self._app_instance = app
        self._config = config
        self.nodes: Dict[str, NodeContext] = {}

    def summarise(self):
        """Produces an immutable description of the system architecture."""
        node_summaries = [node.summarise() for node in self.nodes.values()]
        node_to_summary = {s.fullname: s for s in node_summaries}
        return SystemSummary(node_to_summary)

    def observe(self):
        if self._app_instance.description.distribution.ros == ROSVersion.ROS1:
            observer = ROS1ObserverConnection(self._app_instance, self._config.sources)
        else:
            observer = ROS2ObserverConnection(self._app_instance, self._config.sources)

        nodes = observer.get_nodes()
        self.nodes = nodes
