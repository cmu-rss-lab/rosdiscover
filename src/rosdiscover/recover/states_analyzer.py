# -*- coding: utf-8 -*-
from __future__ import annotations

__all__ = (
    "SymbolicStatesAnalyzer"
)

import typing as t
import attr
from functools import cached_property

from .symbolic import (
    SymbolicAssignment,
    SymbolicFunction,
    SymbolicProgram,
    SymbolicVariableReference,
)
from .analyzer import SymbolicProgramAnalyzer


@attr.s(auto_attribs=True)  # Can't use slots with cached_property
class SymbolicStatesAnalyzer:

    program: SymbolicProgram
    program_analyzer: SymbolicProgramAnalyzer

    @cached_property
    def state_vars(self) -> t.List[SymbolicVariableReference]:
        var_refs: t.List[SymbolicVariableReference] = []
        for pub in self.program_analyzer.publish_calls:
            print(f"Interprodedual conditon of {pub} is: {self.program_analyzer.inter_procedual_condition(pub)}")
            cond = self.program_analyzer.inter_procedual_condition(pub)
            for expr in cond.decendents(True):
                if isinstance(expr, SymbolicVariableReference) and expr.variable in self.program_analyzer.assigned_vars:
                    pub_func = self.program.func_of_stmt(pub)
                    assign_funcs: t.Set[SymbolicFunction] = set()
                    for assign in self.program_analyzer.assignments_of_var(expr.variable):
                        assign_funcs.add(self.program.func_of_stmt(assign))
                    if len(assign_funcs - {pub_func}) > 0 and expr not in var_refs:
                        var_refs.append(expr)
        return var_refs

    def transitions(self) -> t.List[t.Any]:
        result: t.List[t.Any] = []
        return result
