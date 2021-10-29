# -*- coding: utf-8 -*-
__all__ = ('RecoveredNodeModelDatabase',)

import os

from loguru import logger
import attr

from .model import RecoveredNodeModel
from ..config import Config


@attr.s
class RecoveredNodeModelDatabase:
    """Provides an interface for caching recovered node models to and from disk.

    Attributes
    ----------
    path: str
        The absolute path of the database directory on the host file system.

    Raises
    ------
    ValueError
        If the given database path is not an absolute path.
    """
    path: str = attr.ib()

    @path.validator
    def validate_path_must_be_absolute(self, attribute, value) -> None:
        if not os.path.isabs(value):
            message = f"recovered node model database path must be absolute: {value}"
            raise ValueError(message)

    def __attrs_post_init__(self) -> None:
        logger.debug(f"ensuring that model database directory exists: {self.path}")
        os.makedirs(self.path, exist_ok=True)
        logger.debug(f"ensured that model database directory exists: {self.path}")

    def _path(self, image_sha256: str, package_dir: str, node_name: str) -> str:
        """Determines the absolute path of a recovered model on disk.

        Parameters
        ----------
        image_sha256: str
            The SHA256 of the image to which the node belongs, represented as a hex string.
        package_dir: str
            The absolute path of the node's corresponding package directory within its
            associated image.
        node_name: str
            The name of the node.
        """
        # strip the leading / in the package_dir
        assert package_dir[0] == "/"
        package_dir = package_dir[1:]

        rel_path = os.path.join(image_sha256, package_dir, f"{node_name}.json")
        return os.path.join(self.path, rel_path)

    def _path_from_config(
        self,
        config: Config,
        package_name: str,
        node_name: str,
    ) -> str:
        """Determines the absolute path of a recovered model on disk.

        Parameters
        ----------
        config: Config
            the configuration for the system to which the node belongs
        package_name: str
            the name of the package to which the node belongs
        node_name: str
            the name of the node

        Raises
        ------
        ValueError
            if no package exists with the given name inside the specified project
        """
        try:
            package = config.app.description.packages[package_name]
        except KeyError as err:
            message = f"failed to find package with given name: {package_name}"
            raise ValueError(message) from err

        return self._path(config.image_sha256, package.path, node_name)

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
        model_path = self._path_from_config(config, package, node)

        if not os.path.exists(model_path):
            message = (f"failed to fetch recovered model for node [{node}] "
                       f"in package [{package}] for config [{config}]")
            raise ValueError(message)

        return RecoveredNodeModel.load(config, model_path)

    def store(self, model: RecoveredNodeModel) -> None:
        """Stores a given node model in this database."""
        model_path = self._path(model.image_sha256, model.package_abs_path, model.node_name)
        logger.info(f"storing recovered node model [{model}] on disk: {model_path}")
        package_models_dir = os.path.dirname(model_path)
        os.makedirs(package_models_dir, exist_ok=True)
        model.save(model_path)
        logger.info(f"stored recovered node model [{model}] on disk")
