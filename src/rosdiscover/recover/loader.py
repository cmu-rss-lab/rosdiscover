# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = ("SymbolicProgramLoader",)

import typing as t

from .symbolic import (
    SymbolicFunction,
    SymbolicProgram,
)


class SymbolicProgramLoader:
    def _load_function(self, dict_: t.Mapping[str, t.Any]) -> SymbolicFunction:
        raise NotImplementedError

    def load(self, dict_: t.Mapping[str, t.Any]) -> SymbolicProgram:
        functions = [self._load_function(d) for d in dict_["functions"]]
        name_to_function = {function.name: function for function in functions}
        return SymbolicProgram(name_to_function)
