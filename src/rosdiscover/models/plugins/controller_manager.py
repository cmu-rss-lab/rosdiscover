# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ("ControllerManagerPlugin",)

import abc
import typing as t

import attr
from loguru import logger
from roswire.name import namespace_join
from ...interpreter import Interpreter, ModelPlugin, NodeContext


@attr.s(frozen=True, slots=True)
class ControllerManagerPlugin(ModelPlugin, abc.ABC):
    type_name: t.ClassVar[str]

    controller_name: str = attr.ib()
    controller_manager_node: str = attr.ib()

    @classmethod
    def from_type(
        cls,
        controller_type: str,
        controller_name: str,
        controller_manager_node: str,
    ) -> ControllerManagerPlugin:
        """Loads a controller manager plugin.

        Parameters
        ----------
        controller_type: str
            The type of the controller
        controller_name: str
            The name of the controller
        controller_manager_node: str
            The fully qualified name of the node that hosts the associated control manager
        """
        logger.debug(
            f"loading controller_manager plugin [{controller_name}] "
            f"with type [{controller_type}] "
            f"via node [{controller_manager_node}]"
        )

        # find the class associated with this type
        # TODO iterate over subclasses and inspect type_name
        type_to_class: t.Mapping[str, t.Type[ControllerManagerPlugin]] = {
            'joint_state_controller/JointStateController': JointStateControllerPlugin,
            'diff_drive_controller/DiffDriveController': DiffDriveControllerPlugin,
        }
        cls = type_to_class[controller_type]

        plugin = cls.build(controller_name, controller_manager_node)
        logger.debug(f'loaded controller_manager plugin [{controller_name}]')
        return plugin

    @classmethod
    def build(cls, controller_name: str, controller_manager_node: str) -> ControllerManagerPlugin:
        return cls(
            controller_name=controller_name,
            controller_manager_node=controller_manager_node,
        )

    @property
    def namespace(self) -> str:
        """The associated namespace for this controller."""
        return namespace_join(self.controller_manager_node, self.controller_name)

    def load(self, interpreter: Interpreter) -> None:
        context = interpreter.nodes[self.controller_manager_node]
        self._load(interpreter, context)

    @abc.abstractmethod
    def _load(self, interpreter: Interpreter, context: NodeContext) -> None:
        """Simulates the architectural effects of this controller.

        Parameters
        ----------
        interpreter: Interpreter
            provides access to the simulated architecture for the entire ROS system
        context: NodeContext
            provides access to the node context for the associated controller_manager node
            that hosts this controller
        """
        ...


@attr.s(auto_attribs=True, frozen=True, slots=True)
class JointStateControllerPlugin(ControllerManagerPlugin):
    ...


@attr.s(auto_attribs=True, frozen=True, slots=True)
class DiffDriveControllerPlugin(ControllerManagerPlugin):
    ...
