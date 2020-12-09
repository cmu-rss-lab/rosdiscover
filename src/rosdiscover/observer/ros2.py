# -*- coding: utf-8 -*-
from .observer import Observer
from ..interpreter import SystemSummary


class ROS2Observer(Observer):

    def observe(self) -> SystemSummary:
        raise NotImplementedError
