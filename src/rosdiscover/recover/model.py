# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ("RecoveredNodeModel",)

import json
import typing as t

from loguru import logger
import attr

from .loader import SymbolicProgramLoader
from .symbolic import SymbolicProgram
from ..interpreter import NodeModel, NodeContext

from ..config import Config


@attr.s(frozen=True, slots=True, auto_attribs=True)
class CMakeListsInfo:
    filename: str
    lineno: int

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {
            "filename": self.filename,
            "lineno": self.lineno
        }

    @classmethod
    def from_dict(cls, dict_: t.Dict[str, t.Any]) -> "CMakeListsInfo":
        assert 'filename' in dict_ and 'lineno' in dict_
        return CMakeListsInfo(filename=dict_["filename"], lineno=dict_["lineno"])


@attr.s(frozen=True, slots=True, auto_attribs=True)
class RecoveredNodeModel(NodeModel):
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
    cmakelist_info: t.Optional[CMakeListsInfo]
        Information about which CMakeFile the sources came from
    program: SymbolicProgram
        A parameterized, executable description of the node's architectural effects.
    """
    image_sha256: str
    package_name: str
    package_abs_path: str
    source_paths: t.Collection[str]
    node_name: str
    cmakelist_info: t.Optional[CMakeListsInfo]
    program: SymbolicProgram

    def save(self, filename: str) -> None:
        dict_ = self.to_dict()
        logger.debug(f"converted model to JSON-ready dict: {dict_}")
        with open(filename, "w") as f:
            json.dump(dict_, f, indent=2)

    def to_dict(self) -> t.Dict[str, t.Any]:
        dict_ = {
            "image": {
                "sha256": self.image_sha256,
            },
            "node-name": self.node_name,
            "package": {
                "name": self.package_name,
                "path": self.package_abs_path,
            },
            "sources": list(self.source_paths),
            "program": self.program.to_dict(),
        }
        if self.cmakelist_info:
            dict_["cmakeinfo"] = self.cmakelist_info.to_dict()
        return dict_

    @classmethod
    def load(cls, config: Config, filename: str) -> "RecoveredNodeModel":
        with open(filename, "r") as f:
            return cls.from_dict(config, json.load(f))

    @classmethod
    def from_dict(
        cls,
        config: Config,
        dict_: t.Dict[str, t.Any],
    ) -> "RecoveredNodeModel":
        image_sha256 = dict_["image"]["sha256"]
        node_name = dict_["node-name"]
        package_name = dict_["package"]["name"]
        package_abs_path = dict_["package"]["path"]
        source_paths = dict_["sources"]
        program = SymbolicProgramLoader.for_config(config).load(dict_["program"])
        cmakeinfo = None
        if 'cmakeinfo' in dict_:
            cmakeinfo = dict_["cmakeinfo"]
        return RecoveredNodeModel(
            image_sha256=image_sha256,
            node_name=node_name,
            package_name=package_name,
            package_abs_path=package_abs_path,
            source_paths=source_paths,
            program=program,
            cmakelist_info=CMakeListsInfo.from_dict(cmakeinfo) if cmakeinfo else None
        )

    def eval(self, context: NodeContext) -> None:
        context.mark_recovered()
        return self.program.eval(context)
