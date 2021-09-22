# -*- coding: utf-8 -*-
from dockerblade.popen import Popen

from .observer import Observer
from ..interpreter import SystemSummary


class ROS2Observer(Observer):

    def observe(self) -> SystemSummary:
        raise NotImplementedError()

    def execute_script(self, path_on_host: str) -> Popen:
        raise NotImplementedError()
