# -*- coding: utf-8 -*-
__all__ = ("Observer",)

import os
from abc import ABC, abstractmethod
import typing

import roswire
from dockerblade.popen import Popen
from loguru import logger
from roswire import AppInstance, ROSVersion

from ..interpreter import SystemSummary

if typing.TYPE_CHECKING:
    from ..config import Config


class Observer(ABC):

    @classmethod
    def for_container(cls,
                      container: str,
                      config: 'Config',
                      ) -> 'Observer':
        """Constructs an Observer for a given running container.

        Parameters
        ----------
        container: str
            The image id or name of a container running a ROS system.
        config: Config
            A description of the configuration used by the running container.

        Returns
        -------
        Observer
            An observer that is appropriate for the kind of ROS system that is running in the
            container.
        """
        rsw = roswire.ROSWire()
        app = rsw.app(config.image, config.sources)
        instance = app.attach(container, require_description=True)
        if app.description.distribution.ros == ROSVersion.ROS1:
            from .ros1 import ROS1Observer
            return ROS1Observer(instance, config)
        else:
            from .ros2 import ROS2Observer
            return ROS2Observer(instance, config)

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

    def execute_script(self, path_on_host: str) -> Popen:
        """Executes a script on the executing container.

        Parameters
        ----------
        path_on_host: str
            The path to the script on the host (commands in the script should
            be relative to the container, not the host).

        Returns
        -------
        Popen
            The process instance that was started on the container
        """
        if not os.path.exists(path_on_host):
            raise FileNotFoundError(f"'{path_on_host}' not found.")
        assert self._app_instance is not None

        path_on_container = self._app_instance.files.mktemp('.sh')
        self._app_instance.files.copy_from_host(path_on_host, path_on_container)

        cmd = f"bash {path_on_container}"

        logger.debug(f"Running the script in the container: {cmd}")
        process = self._app_instance.shell.popen(cmd)
        return process
