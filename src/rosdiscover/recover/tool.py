# -*- coding: utf-8 -*-
from typing import Iterator
import contextlib

from loguru import logger
from roswire import ROSWire
import attr
import roswire as _roswire

from .core import RecoveredNodeModel
from .python import PythonModelExtractor
from ..config import Config


@attr.s(slots=True, frozen=True, auto_attribs=True)
class RecoveryTool:
    _config: Config
    _system: _roswire.AppInstance = attr.ib(repr=False)

    @classmethod
    @contextlib.contextmanager
    def for_config(cls, config: Config) -> Iterator['RecoveryTool']:
        roswire = ROSWire()
        with roswire.launch(config.image, config.sources) as app:
            yield cls.for_app_instance(config, app)

    @classmethod
    def for_app_instance(cls,
                         config: Config,
                         app: _roswire.System) -> 'RecoveryTool':
        return RecoveryTool(config=config, system=app)

    # TODO update to use latest changes in ROSWire
    # def recover_node(self, package: str, node: str) -> RecoveredNodeModel:
    #     logger.debug(f'recovering model for node [{node}] '
    #                  f'in package [{package}]')
    #     files = self._system.files
    #     binary_path = self.locate_node_binary(package, node)
    #     first_line = files.read(binary_path).partition('\n')[0]
    #     if 'python' in first_line:
    #         return self.recover_py_node(package, node, binary_path)
    #     else:
    #         return self.recover_cpp_node(package, node, binary_path)

    def recover_cpp_node(self,
                         package: str,
                         node: str,
                         path_binary: str
                         ) -> RecoveredNodeModel:
        """Recovers the model for a C++ node from a given binary."""
        logger.debug(f'recovering model for C++ node [{node}] '
                     f'in package [{package}] '
                     f'with binary [{path_binary}]')
        raise NotImplementedError

    def recover_py_node(self,
                        package: str,
                        node_type: str,
                        path_script: str
                        ) -> RecoveredNodeModel:
        """Recovers the model for a Python node from a given script."""
        logger.debug(f'recovering model for Python node type [{node_type}] '
                     f'in package [{package}] '
                     f'with binary [{path_script}]')
        source = self._system.files.read(path_script)
        return PythonModelExtractor(source).extract()
