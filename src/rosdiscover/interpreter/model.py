# -*- coding: utf-8 -*-
__all__ = ('HandwrittenModel', 'PlaceholderModel', 'model', 'NodeModel')

import abc
import typing as t

from loguru import logger
import attr

from .context import NodeContext


class NodeModel(abc.ABC):
    """
    Provides an executable description of the run-time architecture of a given node type
    that can be used to obtain the actual run-time architectural interactions of a node
    within a concrete context.
    """
    @abc.abstractmethod
    def eval(self, context: NodeContext) -> None:
        """Obtain the concrete definition for a node within a given context."""
        ...


@attr.s(frozen=True, slots=True, auto_attribs=True)
class PlaceholderModel(NodeModel):
    """Used in place of a missing node model. Has no architectural effects."""
    package: str
    name: str

    def eval(self, context: NodeContext) -> None:
        context.mark_placeholder()


@attr.s(frozen=True, slots=True)
class HandwrittenModel(NodeModel):
    """Models the architectural interactions of a node type.

    Attributes
    ----------
    package: str
        The name of the package to which the associated node type belongs.
    name: str
        The name of the node type.
    _definition: t.Callable[[NodeContext], None]
        A callable (e.g., a [lambda] function) that implements the architectural
        semantics of this node type in the form of a function that accepts and
        appropriately mutates a given node context. I.e., this attribute is
        used to provide an implementation of the :meth:`NodeModel.eval` method.
    """
    package: str = attr.ib()
    name: str = attr.ib()
    _definition: t.Callable[[NodeContext], None] = attr.ib()

    _models: t.ClassVar[t.Dict[t.Tuple[str, str], "HandwrittenModel"]] = {}

    @staticmethod
    def register(
        package: str,
        name: str,
        definition: t.Callable[[NodeContext], None],
    ) -> None:
        key = (package, name)
        models = HandwrittenModel._models
        if key in models:
            m = f"model [{name}] already registered for package [{package}]"
            raise Exception(m)
        models[key] = HandwrittenModel(package, name, definition)
        logger.debug(f"registered model [{name}] for package [{package}]")

    @staticmethod
    def exists(package: str, name: str) -> bool:
        """Determines whether there exists a handwritten model for a given node type.

        Parameters
        ----------
        package: str
            The name of the package to which the node belongs.
        name: str
            The name of the node type.

        Returns
        -------
        bool
            :code:`True` if there is a handwritten model for the given, else :code:`False`.
        """
        return (package, name) in HandwrittenModel._models

    @staticmethod
    def fetch(package: str, name: str) -> NodeModel:
        """Fetches the prewritten model for a given node.

        Raises
        ------
        ValueError
            if no handwritten model for the given node exists.
        """
        package_and_name = (package, name)
        if package_and_name not in HandwrittenModel._models:
            msg = f"no handwritten model exists for node [{name}] in package [{package}]"
            raise ValueError(msg)

        return HandwrittenModel._models[package_and_name]

    def eval(self, context: NodeContext) -> None:
        try:
            self._definition(context)
            context.mark_handwritten()
        except Exception:
            logger.error(f'Failed to load handwritten model {context.fullname} from {context.launch_filename}')
            raise


def model(package: str, name: str) -> t.Any:
    def register(m: t.Callable[[NodeContext], None]) -> t.Any:
        HandwrittenModel.register(package, name, m)
        return m
    return register
