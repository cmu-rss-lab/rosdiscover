# -*- coding: utf-8 -*-
__all__ = ("ProjectModels",)

import attr

from ..config import Config
from ..interpreter import NodeModel, HandwrittenModel
from ..recover.database import RecoveredModelDatabase


@attr.s(slots=True, auto_attribs=True)
class ProjectModels:
    """Provides an interface for accessing and recovering the models for a given project.

    Attributes
    ----------
    config: Config
        The configuration for the project
    _recovered_models: RecoveredModelDatabase
        A database of all of the models that have been recovered using ROSDiscover
    allow_placeholders: bool
        If :code:`True`, then a :class:`PlaceholderModel` will be used whenever a model
        is missing and irrecoverable for a given node type. If :code:`False`, then a
        :class:`ValueError` will be thrown whenever there is no model available.
    """
    config: Config
    _recovered_models: RecoveredModelDatabase
    allow_placeholders: bool = attr.ib(default=True)
    # TODO add: prefer_handwritten_models attribute

    def recover(self, package: str, node: str) -> NodeModel:
        # FIXME
        if self._recovered_models.contains(self.config, package, node):
            return self._recovered_models.fetch(self.config, package, node)

    def fetch(self, package: str, node: str) -> NodeModel:
        """Retrieves the model for a given node.

        Parameters
        ----------
        package: str
            The name of the package to which the node belongs.
        node: str
            The name of the type of node.

        Returns
        -------
        NodeModel
            When searching for a given model, highest preference will be given to
            handwritten models. If there is no recovered model, then this method will
            attempt to return a handwritten model. If there is also no handwritten model,
            then a placeholder model will be returned instead.

        Raises
        ------
        ValueError
            If there is no model for the given node and placeholders are disabled.
        """
        if HandwrittenModel.exists(package, node):
            return HandwrittenModel.fetch(package, node)

        # TODO check allow placeholders
        return PlaceholderModel(package, node)
