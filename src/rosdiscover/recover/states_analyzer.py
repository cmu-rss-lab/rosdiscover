# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "SymbolicStatesAnalyzer"
)

import typing as t
import attr
from functools import cached_property

from .symbolic import (
    SymbolicProgram,
    SymbolicValue,
    SymbolicVariableReference,
)
from .analyzer import SymbolicProgramAnalyzer


@attr.s(auto_attribs=True, slots=True)
class SymbolicStatesAnalyzer:

    program: SymbolicProgram
    program_analyzer: SymbolicProgramAnalyzer

    @cached_property
    def state_vars(self) -> t.List[SymbolicVariableReference]:
        var_refs: t.List[SymbolicVariableReference] = []
        for pub in self.program_analyzer.publish_calls:
            for expr in pub.condition.decendents():
                if isinstance(expr, SymbolicVariableReference):
                    var_refs.append(expr)

        return var_refs

    @cached_property
    def transitions(self) -> t.List[t.Any]:
        result: t.List[t.Any] = []
        return result
