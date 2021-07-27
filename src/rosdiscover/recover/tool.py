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
    _app_instance: roswire.app.AppInstance = attr.ib(init=False, repr=False)

    @contextlib.contextmanager
    @classmethod
    def for_config(cls, config: Config) -> "NodeRecoveryTool":
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
        return

    def open(self) -> None:
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

    def close(self) -> None:
        self._app_instance.close()

    def recover(self, node_type: str, package: str) -> None:
        raise NotImplementedError
