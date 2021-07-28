# -*- coding: utf-8 -*-
__all__ = ('NodeRecoveryTool',)

import contextlib
import shlex
import subprocess
import os
import types
import typing as t

from loguru import logger
import attr
import roswire

from ..config import Config


@attr.s(auto_attribs=True)
class NodeRecoveryTool:
    _app: roswire.app.App
    _app_instance: t.Optional[roswire.app.AppInstance] = attr.ib(default=None, repr=False)

    @classmethod
    @contextlib.contextmanager
    def for_config(cls, config: Config) -> t.Iterator["NodeRecoveryTool"]:
        with NodeRecoveryTool(app=config.app) as tool:
            yield tool

    def __enter__(self) -> "NodeRecoveryTool":
        self.open()
        return self

    def __exit__(
        self,
        ex_type: t.Optional[t.Type[BaseException]],
        ex_val: t.Optional[BaseException],
        ex_tb: t.Optional[types.TracebackType],
    ) -> None:
        self.close()

    def open(self) -> None:
        if self._app_instance:
            raise ValueError("tool has already been started")

        logger.debug("launching container for static recovery")
        volumes = {
            "rosdiscover-cxx-extract-opt": {
                "mode": "ro",
                "bind": "/opt/rosdiscover",
            },
            "rosdiscover-cxx-extract-llvm": {
                "mode": "ro",
                "bind": "/opt/llvm11",
            },
        }
        self._app_instance = self._app.launch(
            volumes=volumes,
        )
        logger.debug("launched static recovery container")

    def close(self) -> None:
        if not self._app_instance:
            raise ValueError("tool has not been started")

        self._app_instance.close()
        self._app_instance = None

    # TODO we probably need to run this on all C/C++ source files within the workspace and
    # not just the translation unit source files (e.g., header files)
    def _prepare_source_file(self, abs_path: str) -> None:
        """Prepares a source file for static recovery."""
        shell = self._app_instance.shell
        escaped_abs_path = shlex.quote(abs_path)
        shell.run(f'sed -i "s#std::isnan#__STDISNAN__#g" {escaped_abs_path}')
        shell.run(f'sed -i "s#isnan#__STDISNAN__#g" {escaped_abs_path}')
        shell.run(f'sed -i "s#__STDISNAN__#std::isnan#g" {escaped_abs_path}')

    def recover(
        self,
        workspace_abs_path: str,
        source_file_abs_paths: t.Collection[str],
    ) -> None:
        """Statically recovers the dynamic architecture of a given node.

        Parameters
        ----------
        workspace_abs_path: str
            The absolute path to the Catkin workspace (within the container)
            where the source code is located
        source_file_abs_paths: str
            A list of the C++ translation unit source files (i.e., .cpp files)
            for the given node, provided as absolute paths within the container
        """
        if not self._app_instance:
            raise ValueError("tool has not been started")

        logger.debug("beginning static recovery process")
        shell = self._app_instance.shell
        files = self._app_instance.files

        if not source_file_abs_paths:
            raise ValueError("expected at least one source file")

        if not os.path.isabs(workspace_abs_path):
            raise ValueError(f"expected absolute workspace path: {workspace_abs_path}")

        if not files.isdir(workspace_abs_path):
            raise ValueError(f"no directory found at given workspace path: {workspace_abs_path}")

        for source_file in source_file_abs_paths:
            if not os.path.isabs(source_file):
                raise ValueError(f"expected absolute source file path: {source_file}")
            if not files.exists(source_file):
                raise ValueError(f"source file was not found: {source_file}")

        for source_file in source_file_abs_paths:
            self._prepare_source_file(source_file)

        env = {
            "PATH": "/opt/rosdiscover/bin:/opt/llvm11/bin:${PATH:-}",
            "LIBRARY_PATH": "/opt/rosdiscover/lib:/opt/llvm11/lib:${LIBRARY_PATH:-}",
            "LD_LIBRARY_PATH": "/opt/rosdiscover/lib:/opt/llvm11/lib:${LD_LIBRARY_PATH:-}",
        }
        env_args = [f"{var}={val}" for (var, val) in env.items()]
        args = env_args + [
            "rosdiscover",
            "-p",
            shlex.quote(workspace_abs_path),
            ' '.join(shlex.quote(p) for p in source_file_abs_paths),
        ]
        args_s = ' '.join(args)
        logger.debug(f"running static recovery command: {args_s}")
        outcome = shell.run(args_s, text=True, stderr=True)
        logger.debug(f"static recovery output: {outcome.output}")
        logger.debug("finished static recovery process")
