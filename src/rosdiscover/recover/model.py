# -*- coding: utf-8 -*-
__all__ = ("RecoveredNodeModel",)

import json
import typing as t

import attr

from ..interpreter import NodeContext


@attr.s(frozen=True, slots=True)
class RecoveredNodeModel:
    """Provides a symbolic description of the (statically recovered) run-time
    architecture of a given node. This description can be executed via the symbolic
    interpreter to obtain the concrete run-time architecture for a given configuration.

    Attributes
    ----------
    image_sha256: str
        The SHA256 of the image from which this node model was recovered, represented
        as a hexadecimal string.
    package_name: str
        The name of the package to which this node belongs.
    package_abs_path: str
        The absolute path of the directory for the node's package.
    source_paths: t.Collection[str]
        The paths of the source files for this node, relative to the package
        directory,
    node_name: str
        The name of the node.
    """
    image_sha256: str
    package_name: str
    package_abs_path: str
    source_paths: t.Collection[str] = attr.ib(converter=tuple)
    node_name: str

    def save(self, filename: str) -> None:
        dict_ = self.to_dict()
        with open(filename, "w") as f:
            json.dump(dict_, f)

    def to_dict(self) -> t.Dict[str, t.Any]:
        raise NotImplementedError

    @classmethod
    def load(cls, filename: str) -> "RecoveredNodeModel":
        with open(filename, "r") as f:
            return cls.from_dict(json.load(f))

    @classmethod
    def from_dict(cls, dict_: t.Dict[str, t.Any]) -> "RecoveredNodeModel":
        raise NotImplementedError

    def eval(self, context: NodeContext) -> None:
        raise NotImplementedError
