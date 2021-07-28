# -*- coding: utf-8 -*-
__all__ = ('NodeRecoveryTool',)

import contextlib
import shlex
import subprocess
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
        #environment = {
        #    "PATH": "/opt/rosdiscover/bin:/opt/llvm11/bin:${PATH}",
        #}
        environment = {}
        self._app_instance = self._app.launch(
            volumes=volumes,
            environment=environment,
        )
        logger.debug("launched static recovery container")

    def close(self) -> None:
        if not self._app_instance:
            raise ValueError("tool has not been started")

        self._app_instance.close()
        self._app_instance = None

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

        # TODO check that the workspace is an absolute path and actually exists

        # TODO check that the source files are absolute paths and all of them exist

        logger.debug("beginning static recovery process")
        shell = self._app_instance.shell
        args = (
            "rosdiscover",
            "-p",
            shlex.quote(workspace_abs_path),
            ' '.join(shlex.quote(p) for p in source_file_abs_paths),
        )
        args_s = ' '.join(args)
        logger.debug(f"running static recovery command: {args_s}")
        outcome = shell.run(args_s, text=True, stderr=True)
        logger.debug(f"static recovery output: {outcome.output}")
        logger.debug("finished static recovery process")
