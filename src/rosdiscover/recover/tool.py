# -*- coding: utf-8 -*-
__all__ = ('NodeRecoveryTool',)

import attr
import roswire

from .core import RecoveredNodeModel
from .config import Config


@attr.s(frozen=True, auto_attribs=True)
class NodeRecoveryTool:
    _app: roswire.app.App

    @classmethod
    def for_config(cls, config: Config) -> 'NodeRecoveryTool':
        return NodeRecoveryTool(app=config.app)

    def recover(self, node_type: str, package: str) -> RecoveredNodeModel:
        raise NotImplementedError
