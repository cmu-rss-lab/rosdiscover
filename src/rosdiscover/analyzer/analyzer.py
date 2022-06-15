# -*- coding: utf-8 -*-
from typing import Dict, Iterator, Optional
import contextlib
import types
import typing as t

from loguru import logger
import roswire
from roswire import AppInstance, ROSVersion
from roswire.common.launch.config import NodeConfig
from roswire.ros1.launch.reader import ROS1LaunchFileReader
from roswire.ros2.launch.reader import ROS2LaunchFileReader

from ..config import Config
from ..launch import Launch
from ..project import ProjectModels


class Analyzer:
    """
    Attributes
    ----------
    params: ParameterServer
        The simulated parameter server for this analyzer.
    """
    @classmethod
    @contextlib.contextmanager
    def for_config(cls,
                   config: Config
                   ) -> 'Analyzer':
        """Constructs an analyzer for a given configuration"""
        with Analyzer(config) as analyzer:
            yield analyzer

    def __init__(
        self,
        config: Config,
    ) -> None:
        self.models = ProjectModels(config, allow_recovery=False)

    def open(self) -> None:
        self.models.open()

    def analyze(self, pkg_name:str, node_name:str):
        model = self.models.fetch(pkg_name, node_name)
        print(model)

    def close(self) -> None:
        self.models.close()

    def __enter__(self) -> "Analyzer":
        self.open()
        return self

    def __exit__(
        self,
        ex_type: t.Optional[t.Type[BaseException]],
        ex_val: t.Optional[BaseException],
        ex_tb: t.Optional[types.TracebackType],
    ) -> None:
        self.close()