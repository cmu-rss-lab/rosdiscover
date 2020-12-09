# -*- coding: utf-8 -*-
__all__ = ("Observer",)

import contextlib
from abc import ABC, abstractmethod
import typing
from typing import Iterator

import roswire
from roswire import AppInstance, ROSVersion

from ..interpreter import SystemSummary

if typing.TYPE_CHECKING:
    from ..config import Config


class Observer(ABC):

    @classmethod
    @contextlib.contextmanager
    def for_container(cls,
                      container: str,
                      config: 'Config',
                      ) -> Iterator['Observer']:
        """Constructs and interpreter for a given running container

        Parameters
        ----------
        container: str
            The image id or name of a container running a ROS system
        config: Config
            The configuration information that gives information about how to set up the
            environment.


        Returns
        -------
        Iterator[Observer]
            An observer that is appropriate for the kind of ROS system that is running in the
            container.
        """
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
    def __init__(self, app: AppInstance, config: 'Config') -> None:
        self._app_instance = app
        self._config = config

    @abstractmethod
    def observe(self) -> SystemSummary:
        """Dynamically observe the system and produce a summary of the architecture.

        Returns
        -------
        SystemSummmary
            An immutable representation of the system architecture.
        """
        ...
