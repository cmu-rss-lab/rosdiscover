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
        return


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

    # TODO move this logic into ProjectModels class
    @staticmethod
    def find(package: str, name: str) -> NodeModel:
        if (package, name) in HandwrittenModel._models:
            return HandwrittenModel._models[(package, name)]
        else:
            m = (f"failed to find model for node type [{name}] "
                 f"in package [{package}]")
            logger.warning(m)

            return PlaceholderModel(package, name)

    def eval(self, context: NodeContext) -> None:
        return self._definition(context)


def model(package: str, name: str) -> t.Any:
    def register(m: t.Callable[[NodeContext], None]) -> t.Any:
        HandwrittenModel.register(package, name, m)
        return m
    return register
