# -*- coding: utf-8 -*-
__all__ = ('ModelPlugin',)

import abc
import typing

import attr
from loguru import logger

from . import NodeContext

if typing.TYPE_CHECKING:
    from .interpreter import Interpreter


class ModelPlugin(abc.ABC):
    """Models the architectural effects of a dynamically-loaded node plugin
    (e.g., a Gazebo plugin)."""
    @abc.abstractmethod
    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        """Simulates the effects of loading this plugin in a given context."""
        ...


@attr.s(frozen=True, slots=True)
class DynamicPlugin(abc.ABC):
    """
    Models dynamically loaded plygins, taking in a name of the plygin that
    will be recovered when the plugin is loaded
    """

    name: str = attr.ib()

    def load(self, interpreter: 'Interpreter', context: NodeContext) -> None:
        logger.warning("Dynamic loading of plugins for {name} not implemented")
