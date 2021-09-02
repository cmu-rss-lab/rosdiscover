# -*- coding: utf-8 -*-
import enum
import os
import re
import typing as t
from pathlib import Path

import attr
from dockerblade import FileSystem
from loguru import logger
from roswire.common import Package

from .cmake_parser import ParserContext, argparse as cmake_argparse
from .nodelet_xml import NodeletInfo


class Language(enum.Enum):
    CXX = "cxx"
    PYTHON = "python"


@attr.s(auto_attribs=True, slots=True)
class ExecutableInfo:
    name: t.Optional[str]
    language: Language
    sources: t.Collection[str] = attr.ib(converter=frozenset)

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"name": self.name,
                "language": self.language.value,
                "sources": list(self.sources)}

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> "ExecutableInfo":
        return ExecutableInfo(info["name"], Language(info["language"]), set(info["sources"]))


@attr.s(frozen=True, auto_attribs=True)
class PackageSourceInfo:
    executables: t.Mapping[str, ExecutableInfo] = attr.ib(factory=dict)
    entrypoints: t.Mapping[str, str] = attr.ib(factory=dict)

    @classmethod
    def from_package(cls, package: Package, files: FileSystem) -> 'PackageSourceInfo':
        executables = cls.get_sources_from_cmake(files, package)
        entrypoints = cls.get_entrypoints(files, package, executables)
        return PackageSourceInfo(executables=executables, entrypoints=entrypoints)

    @classmethod
    def get_entrypoints(
        cls,
        files: FileSystem,
        package: Package,
        executables: t.Dict[str, ExecutableInfo]
    ) -> t.Dict[str, str]:
        entrypoints: t.Dict[str, str] = dict()
        workspace = package.path
        nodelets_xml_path = os.path.join(workspace, 'nodelet_plugins.xml')
        if files.exists(nodelets_xml_path):
            nodelet_info = NodeletInfo.from_nodelet_xml(files.read(nodelets_xml_path))
            for info in nodelet_info.libraries:
                package_and_name = info.class_name.split('/')
                # TODO can package in XML nodelet be different to package.name?
                package = package_and_name[0]
                name = package_and_name[1]
                entrypoint = info.class_type + "::onInit"
                entrypoints[name] = entrypoint
        return entrypoints

    @classmethod
    def get_sources_from_cmake(
        cls,
        files: FileSystem,
        package: Package
    ) -> t.Dict[str, ExecutableInfo]:
        workspace = package.path
        cmakelists_path = os.path.join(workspace, 'CMakeLists.txt')
        if not files.exists(cmakelists_path):
            raise FileNotFoundError(
                f"failed to find CMakelists.txt at expected location: {workspace}"
            )
        logger.debug(f"Found {cmakelists_path}")
        file_contents = files.read(cmakelists_path)
        logger.debug(f"{file_contents=}")
        executables = cls.process_cmake_contents(file_contents)
        return executables

    @classmethod
    def process_cmake_contents(cls, file_contents):
        executables: t.Dict[str, ExecutableInfo] = dict()
        cmake_env: t.Dict[str, str] = dict({})
        for cmd, args, arg_tokens, (fname, line, column) in ParserContext().parse(file_contents, skip_callable=False):
            if cmd == "set":
                opts, args = cmake_argparse(args, {"PARENT_SCOPE": "-", "FORCE": "-", "CACHE": "*"})
                cmake_env[args[0]] = ";".join(args[1:])
            if cmd == "unset":
                opts, args = cmake_argparse(args, {"CACHE": "-"})
                cmake_env[args[0]] = ""
            if cmd == "add_executable":
                name = args[0]
                sources: t.Set[str] = set()
                # for token_type, token_val in arg_tokens[1:]:
                #     if not token_val.startswith("$"):
                #         sources.add(token_val)
                #     else:
                #         matches = re.match(r'\$\{(.*)\}', token_val)
                #         if matches and matches.group(1) in cmake_env:
                #             sources.update(cmake_env[matches.group(1)].split(";"))
                for source in args[1:]:
                    sources.add(source)
                executables[name] = ExecutableInfo(name=name, language=Language.CXX, sources=sources)
            if cmd == "catkin_install_python":
                opts, args = cmake_argparse(args, {"PROGRAMS": "*", "DESTINATION": "*"})
                if 'PROGRAMS' in opts:
                    for i in range(len(opts['PROGRAMS'])):
                        # http://docs.ros.org/en/jade/api/catkin/html/howto/format2/installing_python.html
                        # Convention is that ros python nodes are in nodes/ directory. All others are in
                        # scripts/. So just include python installs that are in nodes/
                        program = opts['PROGRAMS'][i]
                        if program.startswith("nodes/"):
                            name = Path(program[0]).stem
                            sources = set(program)
                            executables[name] = ExecutableInfo(name, Language.PYTHON, sources)
                else:
                    raise ValueError('PROGRAMS not specified in catin_install_python')
        return executables


