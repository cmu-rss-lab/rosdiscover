# -*- coding: utf-8 -*-
"""
This module is used to provide partial architecture recovery capabilities.
That is, it can be used to produce a (partial) model for a given node by
performing a simple static analysis of its associated source code. This code
operates under fairly heavy restrictions and is most successful when applied
to simple nodes with static models (rather than nodes that may publish and
subscribe to different topics at run-time depending on their configuration).
"""
from typing import Iterator, Optional
import contextlib
import os
import shlex
import subprocess

from comby import Comby
from loguru import logger
from roswire import ROSWire
import attr
import roswire as _roswire

from .core import RecoveredNodeModel
from .cpp import CppModelExtractor
from .python import PythonModelExtractor
from ..config import Config


@attr.s(slots=True, frozen=True, auto_attribs=True)
class RecoveryTool:
    _config: Config
    _system: _roswire.System = attr.ib(repr=False)

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

    def locate_node_binary(self, package: str, node: str) -> str:
        """Attempts to locate the binary for a given node.

        Returns
        -------
        str
            The absolute path of the binary for that node.

        Raises
        ------
        ValueError
            If the given package could not be found.
        ValueError
            If no binary can be located for the given node.
        """
        path: Optional[str] = None
        logger.debug(f'locating binary for node [{node}] '
                     f'in package [{package}]')
        shell = self._system.shell
        files = self._system.files

        packages = self._system.description.packages
        if package not in packages:
            raise ValueError(f"package not found: {package}")
        package_dir = packages[package].path

        # start by looking in libexec
        command = ('catkin_find --first-only --libexec '
                   f'{shlex.quote(package)} {shlex.quote(node)}')
        try:
            path = shell.check_output(command, stderr=False)
        except subprocess.CalledProcessError as err:
            pass

        # look in the scripts directory of the package's source directory
        path_in_scripts_dir = os.path.join(package_dir, 'scripts', node)
        if files.isfile(path_in_scripts_dir):
            if files.access(path_in_scripts_dir, os.X_OK):
                path = path_in_scripts_dir

        if not path:
            m = (f"unable to locate binary for node [{node}] "
                 f"in package [{package}]")
            raise ValueError(m)

        logger.debug(f'located binary for node [{node}] '
                     f'in package [{package}]: {path}')
        return path

    def recover_node(self, package: str, node: str) -> RecoveredNodeModel:
        logger.debug(f'recovering model for node [{node}] '
                     f'in package [{package}]')
        files = self._system.files
        binary_path = self.locate_node_binary(package, node)
        first_line = files.read(binary_path).partition('\n')[0]
        if 'python' in first_line:
            return self.recover_py_node(package, node, binary_path)
        else:
            return self.recover_cpp_node(package, node, binary_path)

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
                        node: str,
                        path_script: str
                        ) -> RecoveredNodeModel:
        """Recovers the model for a Python node from a given script."""
        logger.debug(f'recovering model for Python node [{node}] '
                     f'in package [{package}] '
                     f'with binary [{path_script}]')
        source = self._system.files.read(path_script)
        return PythonModelExtractor(source).extract()
