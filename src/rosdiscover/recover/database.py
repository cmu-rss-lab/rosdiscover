# -*- coding: utf-8 -*-
__all__ = ('RecoveredNodeModelDatabase',)

import os
import typing as t

import attr

from .model import RecoveredNodeModel


@attr.s(auto_attribs=True)
class RecoveredNodeModelDatabase:
    """Provides an interface for caching recovered node models to and from disk.

    Attributes
    ----------
    path: str
        The absolute path of the database directory on the host file system.
    """
    path: str

    def _path(self, image_sha1: str, package_dir: str, node_name: str) -> str:
        """Determines the absolute path of a recovered model on disk.

        Parameters
        ----------
        image_sha1: str
            The SHA1 of the image to which the node belongs, represented as a hex string.
        package_dir: str
            The absolute path of the node's corresponding package directory within its
            associated image.
        node_name: str
            The name of the node.
        """
        # strip the leading / in the package_dir
        assert package_dir[0] == "/"
        package_dir = package_dir[1:]

        rel_path = os.path.join(image_sha1, package_dir, f"{node_name}.json")
        return os.path.join(self.path, rel_path)

    def _path_from_config(self, config: Config, package: str, node: str) -> str:
        """Determines the absolute path of a recovered model on disk.

        Parameters
        ----------
        config: Config
            the configuration for the system to which the node belongs
        package: str
            the name of the package to which the node belongs
        node: str
            the name of the node
        """
        raise NotImplementedError

    def contains(self, config: Config, package: str, node: str) -> bool:
        """Determines whether this database contains a recovered model for a given node.

        Parameters
        ----------
        config: Config
            the configuration for the system to which the node belongs
        package: str
            the name of the package to which the node belongs
        node: str
            the name of the node

        Returns
        -------
        bool
            True if the database contains a recovered model, or False if it does not.
        """
        model_path = self._path_from_config(config, package, node)
        return os.path.exists(model_path)

    def fetch(self, config: Config, package: str, node: str) -> RecoveredNodeModel:
        """Retrieves the model of a given node.

        Parameters
        ----------
        config: Config
            the configuration for the system to which the node belongs
        package: str
            the name of the package to which the node belongs
        node: str
            the name of the node

        Raises
        ------
        ValueError
            if no model exists for the given node
        """
        raise NotImplementedError

    def store(self, model: RecoveredNodeModel) -> None:
        """Stores a given node model in this database."""
        model_path = self._path(model.image_sha1, model.package_dir, model.node_name)
        package_models_dir = os.path.dirname(abs_path)
        os.makedirs(package_models_dir, exist_ok=True)
        model.save(model_path)
