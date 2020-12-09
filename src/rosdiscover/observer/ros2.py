# -*- coding: utf-8 -*-
from roswire import AppInstance

from .observer import Observer
from .. import Config
from ..interpreter import SystemSummary


class ROS2Observer(Observer):

    def __init__(self, app: AppInstance, config: Config) -> None:
        super().__init__(app, config)

    def observe(self) -> SystemSummary:
        raise NotImplementedError
