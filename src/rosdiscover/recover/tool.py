# -*- coding: utf-8 -*-
__all__ = ('NodeRecoveryTool',)

import contextlib
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
        self._app_instance = self._app.launch(volumes=volumes)
        logger.debug("launched static recovery container")

    def close(self) -> None:
        if not self._app_instance:
            raise ValueError("tool has not been started")

        self._app_instance.close()
        self._app_instance = None

    def recover(self, node_type: str, package: str) -> None:
        raise NotImplementedError
