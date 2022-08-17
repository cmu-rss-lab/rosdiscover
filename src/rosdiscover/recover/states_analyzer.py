# -*- coding: utf-8 -*-
from __future__ import annotations

from .call import Subscriber

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

    @cached_property
    def _state_var_assigns(self) -> t.List[SymbolicAssignment]:
        result: t.List[SymbolicAssignment] = []
        for var in self.state_vars:
            for assign in self.program_analyzer.assignments_of_var(var.variable):
                if assign not in result:
                    result.append(assign)
        return result

    @cached_property
    def sub_state_var_assigns(self) -> t.List[t.Tuple[Subscriber, SymbolicAssignment]]:
        result: t.List[t.Tuple[Subscriber, SymbolicAssignment]] = []
        for assign in self._state_var_assigns:
            for (sub, callback) in self.program_analyzer.subscriber_callbacks_map:
                if callback.body.contains(assign, self.program.functions) and (sub, assign) not in result:
                    result.append((sub, assign))
        return result