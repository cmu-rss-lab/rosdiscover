# -*- coding: utf-8 -*-
from .observer import Observer
from ..interpreter import SystemSummary


class ROS2Observer(Observer):

    def observe(self) -> SystemSummary:
        raise NotImplementedError

    def launch_from_config(self, sleep_time: float) -> int:
        raise NotImplementedError
