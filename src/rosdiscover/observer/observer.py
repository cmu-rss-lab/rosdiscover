# -*- coding: utf-8 -*-
__all__ = ("Observer",)

import contextlib
from abc import ABC, abstractmethod
import typing
from typing import Dict, Iterator

import roswire
from roswire import AppInstance, ROSVersion

from ..interpreter import NodeContext, SystemSummary

if typing.TYPE_CHECKING:
    from ..config import Config


class Observer(ABC):

    @classmethod
    @contextlib.contextmanager
    def for_container(cls,
                      container: str,
                      config: 'Config',
                      ) -> Iterator['Observer']:
        """Constructs and interpreter for a given running container"""
        rsw = roswire.ROSWire()
        app = rsw.app(config.image, config.sources)
        instance = app.attach(container, require_description=True)
        if app.description.distribution.ros == ROSVersion.ROS1:
            from .ros1 import ROS1Observer
            yield ROS1Observer(instance, config)
        else:
            from .ros2 import ROS2Observer
            yield ROS2Observer(instance, config)

    @abstractmethod
    def __init__(self, app: AppInstance, config: 'Config'):
        self._app_instance = app
        self._config = config
        self._nodes: Dict[str, NodeContext] = {}

    def summarise(self):
        """Produces an immutable description of the system architecture."""
        node_summaries = [node.summarise() for node in self._nodes.values()]
        node_to_summary = {s.fullname: s for s in node_summaries}
        return SystemSummary(node_to_summary)

    @abstractmethod
    def observe_and_summarise(self):
        ...
