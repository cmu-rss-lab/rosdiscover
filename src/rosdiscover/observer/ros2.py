# -*- coding: utf-8 -*-
import typing as t

from dockerblade.popen import Popen

from .observer import Observer
from ..interpreter import SystemSummary


class ROS2Observer(Observer):

    def observe(self) -> SystemSummary:
        raise NotImplementedError

    def launch_from_config(self, sleep_time: float) -> t.Sequence[Popen]:
        raise NotImplementedError
