# -*- coding: utf-8 -*-
__all__ = ('ModelPlugin',)

import abc
import typing

if typing.TYPE_CHECKING:
    from .interpreter import Interpreter


class ModelPlugin(abc.ABC):
    """Models the architectural effects of a dynamically-loaded node plugin
    (e.g., a Gazebo plugin)."""
    @abc.abstractmethod
    def load(self, interpreter: 'Interpreter') -> None:
        """Simulates the effects of loading this plugin in a given context."""
        ...
