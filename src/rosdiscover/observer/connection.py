# -*- coding: utf-8 -*-
from abc import ABC, abstractmethod


class ObserverConnection(ABC):

    @abstractmethod
    def get_nodes(self):
        ...
