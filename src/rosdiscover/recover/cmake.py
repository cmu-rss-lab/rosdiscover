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


class ExecutableKind(enum.Enum):
    NODE = "node"
    LIBRARY = "library"


@attr.s(auto_attribs=True, slots=True)
class ExecutableInfo:
    name: t.Optional[str]
    language: Language
    kind: ExecutableKind
    sources: t.Collection[str] = attr.ib(converter=frozenset)
    restrict_to_paths: t.Collection[str] = attr.ib(converter=frozenset)

    def to_dict(self) -> t.Dict[str, t.Any]:
        return {"name": self.name,
                "language": self.language.value,
                "kind": self.kind.value,
                "sources": list(self.sources),
                "path_restrictions": list(self.restrict_to_paths)}

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> "ExecutableInfo":
        return ExecutableInfo(info["name"],
                              Language(info["language"]),
                              ExecutableKind(info["kind"]),
                              set(info["sources"]),
                              set(info["path_restrictions"]))

    @property
    def entrypoint(self) -> t.Optional[str]:
        if self.language == Language.CXX:
            return "main"
        else:
            return None


@attr.s(auto_attribs=True, slots=True)
class NodeletExecutableInfo(ExecutableInfo):
    entrypoint: str

    def to_dict(self) -> t.Dict[str, t.Any]:
        d = super().to_dict()
        d['entrypoint'] = self.entrypoint
        return d

    @classmethod
    def from_dict(cls, info: t.Dict[str, t.Any]) -> 'NodeletExecutableInfo':
        return NodeletExecutableInfo(info["name"],
                                     Language(info["language"]),
                                     ExecutableKind(info["kind"]),
                                     set(info["sources"]),
                                     set(info["path_restrictions"]),
                                     info['entrypoint'])

    @property
    def entrypoint(self):
        return self.entrypoint


@attr.s(frozen=True, auto_attribs=True)
class PackageSourceInfo:
    executables: t.Mapping[str, ExecutableInfo] = attr.ib(factory=dict)

    @classmethod
    def from_package(cls, package: Package, files: FileSystem) -> 'PackageSourceInfo':
        executables = cls.get_sources_from_cmake(files, package)
        entrypoints = cls.get_entrypoints(files, package, executables)
        logger.debug(f"Processing entrypoints: {entrypoints}")
        for entrypoint in entrypoints:
            logger.debug(f"Processing {entrypoint} {entrypoint in executables}")
            if entrypoint in executables:
                executables[entrypoint] = NodeletExecutableInfo(
                    executables[entrypoint].name,
                    Language.CXX,
                    ExecutableKind.LIBRARY,
                    executables[entrypoint].sources,
                    entrypoints[entrypoint]
                )
        return PackageSourceInfo(executables=executables)

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
        env = {}
        executables = cls.process_cmake_contents(file_contents, files, package, env)
        return executables

    @classmethod
    def process_cmake_contents(cls,
                               file_contents: str,
                               files: FileSystem,
                               package: Package,
                               cmake_env: t.Dict[str, str] = {}) -> t.Dict[str, ExecutableInfo]:
        executables: t.Dict[str, ExecutableInfo] = dict()
        # cmake_env: t.Dict[str, str] = dict({})
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
                    if 'cwd' in cmake_env:
                        sources.add(os.path.join(cmake_env['cwd'], source))
                    else:
                        sources.add(source)
                logger.debug(f"Adding C++ sources for {name}")
                executables[name] = ExecutableInfo(
                    name,
                    Language.CXX,
                    ExecutableKind.NODE,
                    sources,
                    PackageSourceInfo.package_paths(package))
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
                            if 'cwd' in cmake_env:
                                sources = set(os.path.join(cmake_env['cwd'], program))
                            logger.debug(f"Adding Python sources for {name}")
                            executables[name] = ExecutableInfo(name,
                                                               Language.PYTHON,
                                                               ExecutableKind.NODE,
                                                               sources,
                                                               set())
                else:
                    raise ValueError('PROGRAMS not specified in catin_install_python')
            if cmd == 'add_library':
                name = args[0]
                if 'cwd' in cmake_env:
                    sources = {os.path.join(cmake_env['cwd'], s) for s in args[1:]}
                else:
                    sources = set(args[1:])
                logger.debug(f"Adding C++ library {name}")
                executables[name] = ExecutableInfo(
                    name,
                    Language.CXX,
                    ExecutableKind.LIBRARY,
                    sources,
                    PackageSourceInfo.package_paths(package))
            if cmd == "add_subdirectory":
                new_env = cmake_env.copy()
                new_env['cwd'] = os.path.join(cmake_env.get('cwd', '.'), args[0])
                logger.debug(f"Package path: {package.path}")
                logger.debug(f"Env: {new_env['cwd']}")
                join = os.path.join(package.path, new_env['cwd'])
                logger.debug(f"{join=}")
                cmakelists_path = os.path.join(join, 'CMakeLists.txt')
                logger.debug(f"Processing {cmakelists_path}")
                included_package_info = PackageSourceInfo.process_cmake_contents(files.read(cmakelists_path),
                                                                                 files, package, new_env)
                executables = executables | included_package_info
        return executables

    @classmethod
    def package_paths(cls, package):
        # TODO Do this properly
        return {package.path, os.path.normpath(os.path.join(package.path, f'../../include/{package.name}'))}


