# -*- coding: utf-8 -*-
from typing import Iterator
import contextlib
import types
import typing as t

import roswire
from roswire import AppInstance

from ..config import Config
from ..project import ProjectModels
from .summary import StateMachineSummary, State


class StateMachineRecovery:
    """
    Attributes
    ----------
    """
    @classmethod
    @contextlib.contextmanager
    def for_config(cls,
                   config: Config
                   ) -> Iterator['StateMachineRecovery']:
        """Constructs an interpreter for a given configuration"""
        rsw = roswire.ROSWire()  # TODO don't maintain multiple instances
        with rsw.launch(config.image, config.sources, environment=config.environment) as app:
            with StateMachineRecovery(config, app) as recovery:
                yield recovery

    def __init__(
        self,
        config: Config,
        app: roswire.System,
    ) -> None:
        self._app = app
        self.models = ProjectModels(config, allow_recovery=True)

    def open(self) -> None:
        self.models.open()

    def close(self) -> None:
        self.models.close()

    def __enter__(self) -> "StateMachineRecovery":
        self.open()
        return self

    def __exit__(
        self,
        ex_type: t.Optional[t.Type[BaseException]],
        ex_val: t.Optional[BaseException],
        ex_tb: t.Optional[types.TracebackType],
    ) -> None:
        self.close()

    def summarise(self) -> StateMachineSummary:
        initial = State({})
        return StateMachineSummary(
            initial=initial,
            states={initial}
        )

    @property
    def app(self) -> AppInstance:
        return self._app
