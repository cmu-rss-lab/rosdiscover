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
    SymbolicVariableReference,
)
from .analyzer import SymbolicProgramAnalyzer


@attr.s(auto_attribs=True) # Can't use slots with cached_property
class SymbolicStatesAnalyzer:

    program: SymbolicProgram
    program_analyzer: SymbolicProgramAnalyzer

    @cached_property
    def potential_state_vars(self) -> t.List[SymbolicVariableReference]:
        var_refs: t.List[SymbolicVariableReference] = []
        for pub in self.program_analyzer.publish_calls:
            for expr in pub.condition.decendents():
                if isinstance(expr, SymbolicVariableReference) and expr.variable in self.program_analyzer.assigned_vars:
                    pub_func = self.program_analyzer.func_of_stmt(pub)
                    var_assigns = self.program_analyzer.assignments_of_var(expr.variable)
                    assign_funcs = {self.program_analyzer.func_of_stmt(assign) for assign in var_assigns}
                    if len(assign_funcs - pub_func) > 0:
                        var_refs.append(expr)

        return var_refs

    @cached_property
    def transitions(self) -> t.List[t.Any]:
        result: t.List[t.Any] = []
        return result
