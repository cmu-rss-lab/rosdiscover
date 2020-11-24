# -*- coding: utf-8 -*-
from roswire import AppInstance

from .observer import Observer
from .. import Config


class ROS2Observer(Observer):

    def __init__(self, app: AppInstance, config: Config):
        super().__init__(app, config)

    def observe_and_summarise(self):
        pass
