# -*- coding: utf-8 -*-
import contextlib
import os
from typing import Dict, Iterator

import roswire
from roswire import AppInstance, ROSVersion

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
        rsw = roswire.ROSWire()
        container_app = rsw._dockerblade.attach(container)
        instance = AppInstance(None, dockerblade=container_app)
        yield Observer(instance, config)

    def __init__(self, app: roswire.System, config: Config):
        self._app = app
        self._config = config
        self.nodes: Dict[str, NodeContext] = {}

    def summarise(self):
        """Produces an immutable description of the system architecture."""
        node_summaries = [node.summarise() for node in self.nodes.values()]
        node_to_summary = {s.fullname: s for s in node_summaries}
        return SystemSummary(node_to_summary)

    def observe(self):
        if self._app.description.distribution.ros == ROSVersion.ROS1:
            observer = ROS1Observer(self._app, self._config.sources)
        else:
            observer = ROS2Observer(self._app, self._config.sources)

        nodes = observer.get_nodes()
        self.nodes = nodes


