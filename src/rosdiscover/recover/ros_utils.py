# -*- coding: utf-8 -*-
import enum
import re
import typing as t
from pathlib import Path

import attr

from .cmake import ParserContext, argparse as cmake_argparse


class Language(enum.Enum):
    CXX = "cxx"
    PYTHON = "python"


@attr.s(frozen=True, auto_attribs=True, slots=True)
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

    @classmethod
    def from_cmake(cls, file_contents: str) -> t.Collection["ExecutableInfo"]:
        exe_infos = set()
        cmake_env: t.Dict[str, str] = dict({})

        for cmd, args, arg_tokens, (fname, line, column) in ParserContext().parse(file_contents):
            if cmd == "set":
                opts, args = cmake_argparse(args, {"PARENT_SCOPE": "-", "FORCE": "-", "CACHE": "*"})
                cmake_env[args[0]] = ";".join(args[1:])
            if cmd == "unset":
                opts, args = cmake_argparse(args, {"CACHE": "-"})
                cmake_env[args[0]] = ""
            if cmd == "add_executable":
                name = args[0]
                sources: t.Set[str] = set()
                for token_type, token_val in arg_tokens[1:]:
                    if not token_val.startswith("$"):
                        sources.add(token_val)
                    else:
                        matches = re.match(r'\$\{(.*)\}', token_val)
                        if matches and matches.group(1) in cmake_env:
                            sources.update(cmake_env[matches.group(1)].split(";"))
                exe_infos.add(ExecutableInfo(name=name, language=Language.CXX, sources=sources))
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
                            exe_infos.add(ExecutableInfo(name, Language.PYTHON, sources))
                else:
                    raise ValueError('PROGRAMS not specified in catin_install_python')
            else:
                raise ValueError('PROGRAMS not specified in catin_install_python')
        return exe_infos
